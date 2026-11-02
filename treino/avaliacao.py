"""O modelo dirigindo no simulador — a métrica que importa.

Erro médio de validação diz quanto a rede acerta numa imagem; não diz se o
carro chega ao fim da rota. Aqui a rede de faixa fecha a malha, como no carro:

    câmera → rede → filtro (encoders + giroscópio) → PilotoFaixa → rpm → carro

Nesta etapa, **semáforo e manobra vêm da verdade** (leitura perfeita de placa
e semáforo) — isola a rede de faixa. Quando o detector entrar no laço, a
manobra passa a sair de ``rules.decide`` sobre a placa detectada.

Use sementes que NÃO estavam no treino (padrão: a partir de 100000).
"""

from __future__ import annotations

import json
import math
import time
from collections import Counter
from collections.abc import Callable
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from vista_sim.car import RPM
from vista_sim.config import CameraParams, CarParams
from vista_sim.controle import FiltroFaixa, Percepcao, PilotoFaixa
from vista_sim.env import SimConfig, VistaSimEnv
from vista_sim.track import Track

from .rotulos import MANOBRAS

# preditor(imagem_bgr, manobra) → [e_y, e_ψ, κ, dist, p_dentro]
Preditor = Callable[[np.ndarray, str], np.ndarray]


class PreditorOnnx:
    """A rede de faixa em ONNX (o MESMO arquivo que vai para a Jetson, antes
    do TensorRT), via onnxruntime. Sem PyTorch."""

    def __init__(self, caminho: Path | str, threads: int | None = None) -> None:
        import onnxruntime as ort

        caminho = Path(caminho)
        meta = json.loads(caminho.with_suffix(".json").read_text(encoding="utf-8"))
        self.h, self.w = meta["altura"], meta["largura"]
        so = ort.SessionOptions()
        if threads:
            so.intra_op_num_threads = threads
            so.inter_op_num_threads = 1
        prov = [p for p in ("CUDAExecutionProvider", "CPUExecutionProvider") if p in ort.get_available_providers()]
        self.sess = ort.InferenceSession(str(caminho), so, providers=prov)

    def __call__(self, img_bgr: np.ndarray, manobra: str) -> np.ndarray:
        img = cv2.cvtColor(cv2.resize(img_bgr, (self.w, self.h), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)
        x = (img.astype(np.float32) / 255.0).transpose(2, 0, 1)[None]
        m = np.zeros((1, 3), np.float32)
        m[0, MANOBRAS.index(manobra)] = 1.0
        return self.sess.run(None, {"imagem": x, "manobra": m})[0][0]


class PilotoRede:
    """O Vista em miniatura: imagem → rede → filtro → controle → rpm.

    ``filtro=True`` usa a velocidade dos encoders e o giro da MPU6050 (viés
    calibrado no vermelho) da TELEMETRIA — o que o carro real tem.
    """

    def __init__(self, preditor: Preditor, car: CarParams, filtro: bool = True) -> None:
        self.pred = preditor
        self.car = car
        self.piloto = PilotoFaixa(car)
        self.filtro = FiltroFaixa(car) if filtro else None

    def reset(self) -> None:
        self.piloto.reset()
        if self.filtro:
            self.filtro.reset()

    def perceber(self, obs: np.ndarray, tel, manobra: str, dt: float = 1 / 30) -> Percepcao:
        o = self.pred(obs, manobra)
        p = Percepcao(float(o[0]), float(o[1]), float(o[2]), float(o[3]), bool(o[4] > 0.5))
        if self.filtro is not None:
            v = 0.5 * (tel.rpm_left + tel.rpm_right) / RPM * self.car.wheel_radius
            w = tel.gyro_z - tel.gyro_bias_est
            p = self.filtro.atualizar(p, v, w, dt)
        return p

    def act(self, obs: np.ndarray, tel, manobra: str, go: bool, dt: float = 1 / 30):
        p = self.perceber(obs, tel, manobra, dt)
        return self.piloto.act(p, manobra, go, dt), p


def percepcao_da_verdade(tr) -> Percepcao:
    nj = tr.next_junction
    dist = min(max(nj["distance_m"], 0.0), 3.0) if nj else 3.0
    return Percepcao(tr.e_y, tr.e_psi, tr.kappa_ahead, dist, tr.edge is None)


def rodar(preditor: Preditor | None, sementes: list[int], camera: CameraParams | None = None,
          track: Track | None = None, config: SimConfig | None = None, filtro: bool = True,
          log: Callable[[str], Any] = print) -> dict[str, Any]:
    """``preditor=None`` usa a verdade de campo: o teto (e teste do controle)."""
    cfg = config or SimConfig(randomize_fx=True)
    res = []
    for seed in sementes:
        env = VistaSimEnv(track=track, camera=camera, config=cfg, seed=seed)
        track = env.track
        obs, info = env.reset(seed=seed)
        tr, tel = info["truth"], info["telemetry"]
        piloto_v = PilotoFaixa(env.car_params)
        piloto_r = PilotoRede(preditor, env.car_params, filtro) if preditor is not None else None
        erros: list[tuple[float, float]] = []
        ey: list[float] = []
        t0, n = time.perf_counter(), 0
        while True:
            nj = tr.next_junction
            manobra = nj["maneuver"] if nj else "frente"
            go = tr.traffic_light == "verde"
            if piloto_r is None:
                cmd = piloto_v.act(percepcao_da_verdade(tr), manobra, go)
            else:
                cmd, p = piloto_r.act(obs, tel, manobra, go)
                erros.append((p.e_y - tr.e_y, math.remainder(p.e_psi - tr.e_psi, 2 * math.pi)))
            obs, tel, tr, done, inf = env.step(*cmd)
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
        "filtro": filtro if preditor is not None else None,
        "por_rota": res,
    }
