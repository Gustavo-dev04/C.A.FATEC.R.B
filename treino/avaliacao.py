"""O modelo dirigindo no simulador — a métrica que importa.

Erro médio de validação diz quanto a rede acerta numa imagem; não diz se o
carro chega ao fim da rota. Aqui a rede de faixa fecha a malha:

    câmera → rede (e_y, e_ψ, κ, dist, dentro?) → PilotoFaixa → rpm → carro

Nesta etapa, **semáforo e manobra vêm da verdade** (leitura perfeita de placa
e semáforo) — isola a rede de faixa. Quando o detector entrar no laço, a
manobra passa a sair de ``rules.decide`` sobre a placa detectada.

Use sementes que NÃO estavam no treino (padrão: a partir de 100000).
"""

from __future__ import annotations

import math
import time
from collections import Counter
from collections.abc import Callable
from typing import Any

import numpy as np

from vista_sim.config import CameraParams
from vista_sim.controle import Percepcao, PilotoFaixa
from vista_sim.env import SimConfig, VistaSimEnv
from vista_sim.track import Track

# preditor(imagem_bgr, manobra) → [e_y, e_ψ, κ, dist, p_dentro]
Preditor = Callable[[np.ndarray, str], np.ndarray]


def percepcao_da_verdade(tr) -> Percepcao:
    nj = tr.next_junction
    dist = min(max(nj["distance_m"], 0.0), 3.0) if nj else 3.0
    return Percepcao(tr.e_y, tr.e_psi, tr.kappa_ahead, dist, tr.edge is None)


def rodar(preditor: Preditor | None, sementes: list[int], camera: CameraParams | None = None,
          track: Track | None = None, config: SimConfig | None = None,
          log: Callable[[str], Any] = print) -> dict[str, Any]:
    """``preditor=None`` usa a verdade de campo: o teto (e teste do controle)."""
    cfg = config or SimConfig(randomize_fx=True)
    res = []
    for seed in sementes:
        env = VistaSimEnv(track=track, camera=camera, config=cfg, seed=seed)
        track = env.track
        obs, info = env.reset(seed=seed)
        tr = info["truth"]
        piloto = PilotoFaixa(env.car_params)
        erros: list[tuple[float, float]] = []
        ey: list[float] = []
        t0, n = time.perf_counter(), 0
        while True:
            nj = tr.next_junction
            manobra = nj["maneuver"] if nj else "frente"
            if preditor is None:
                p = percepcao_da_verdade(tr)
            else:
                o = preditor(obs, manobra)
                p = Percepcao(float(o[0]), float(o[1]), float(o[2]), float(o[3]), bool(o[4] > 0.5))
                erros.append((p.e_y - tr.e_y, math.remainder(p.e_psi - tr.e_psi, 2 * math.pi)))
            go = tr.traffic_light == "verde"
            obs, _, tr, done, inf = env.step(*piloto.act(p, manobra, go))
            n += 1
            if go:
                ey.append(tr.e_y)
            if done:
                break
        ev = Counter(e["tipo"] for e in env.events)
        r = {
            "semente": seed, "motivo": inf["reason"], "rota_m": round(env.route.length, 1),
            "progresso": round(min(tr.route_s / env.route.length, 1.0), 3),
            "cruzamentos_ok": ev.get("placa_respeitada", 0) + ev.get("cruzamento_sem_placa", 0),
            "cruzamentos": len(env.route.junctions),
            "e_y_rms_cm": round(float(np.sqrt(np.mean(np.square(ey)))) * 100, 2) if ey else None,
            "e_y_max_cm": round(float(np.max(np.abs(ey))) * 100, 2) if ey else None,
            "contramao": ev.get("entrou_na_contramao", 0),
            "tempo_s": round(env.t, 1), "passos_s": round(n / (time.perf_counter() - t0), 1),
        }
        if erros:
            e = np.array(erros)
            r["erro_rede_e_y_cm"] = round(float(np.mean(np.abs(e[:, 0]))) * 100, 2)
            r["erro_rede_e_psi_graus"] = round(math.degrees(float(np.mean(np.abs(e[:, 1])))), 2)
        res.append(r)
        log(f"semente {seed}: {r['motivo']:<18} progresso {r['progresso'] * 100:5.1f}%  "
            f"cruzamentos {r['cruzamentos_ok']}/{r['cruzamentos']}  e_y rms {r['e_y_rms_cm']} cm")
    completas = sum(r["motivo"] == "rota_completa" for r in res)
    return {
        "rotas": len(res), "completas": completas, "taxa_conclusao": round(completas / max(len(res), 1), 3),
        "progresso_medio": round(float(np.mean([r["progresso"] for r in res])), 3),
        "motivos": dict(Counter(r["motivo"] for r in res)),
        "e_y_rms_cm_mediana": float(np.median([r["e_y_rms_cm"] for r in res if r["e_y_rms_cm"] is not None] or [np.nan])),
        "por_rota": res,
    }
