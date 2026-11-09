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


class FaixaVerdade:
    """Rede de faixa PERFEITA (verdade de campo), no formato da saída da rede.
    Antes de cada chamada: ``faixa.tr = truth``. Só para o teto e testes."""

    def __init__(self) -> None:
        self.tr = None

    def __call__(self, img_bgr: np.ndarray, manobra: str) -> np.ndarray:
        p = percepcao_da_verdade(self.tr)
        return np.array([p.e_y, p.e_psi, p.kappa, p.dist_cruzamento, float(p.no_cruzamento)])


class FaixaRuidosa(FaixaVerdade):
    """Verdade + ruído do tamanho do erro da rede — mede a ROBUSTEZ do resto
    do Vista sem depender de um modelo treinado. Mesmo ruído do experimento de
    02/11 (diário): e_y 0,8 cm na faixa / 2,5 cm perto do cruzamento, e_ψ
    0,7° / 1,5°, κ 0,15 1/m; mais 3,6 cm na distância ao cruzamento."""

    def __init__(self, sig_faixa: float = 0.008, sig_cruz: float = 0.025, semente: int = 0) -> None:
        super().__init__()
        self.sig_faixa, self.sig_cruz = sig_faixa, sig_cruz
        self.rng = np.random.default_rng(semente)

    def __call__(self, img_bgr: np.ndarray, manobra: str) -> np.ndarray:
        o = super().__call__(img_bgr, manobra)
        perto = o[4] > 0.5 or o[3] < 0.3
        n = self.rng.normal(0.0, 1.0, 4)
        o[0] += n[0] * (self.sig_cruz if perto else self.sig_faixa)
        o[1] += n[1] * math.radians(1.5 if perto else 0.7)
        o[2] += n[2] * 0.15
        o[3] = max(o[3] + n[3] * 0.036, 0.0)
        return o


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


# --------------------------------------------------------------------------
# O Vista completo: só sensores do carro
# --------------------------------------------------------------------------
#
#   câmera → rede de faixa → filtro → PilotoFaixa          (na faixa)
#   câmera → detector → placa da MINHA aproximação → rules.decide
#   mapa + odometria (encoders + giroscópio) → curva       (no cruzamento)
#
# Semáforo e manobra NÃO vêm mais da verdade. ``faixa=None`` e
# ``detector="verdade"`` trocam cada parte pela verdade de campo, para isolar.

_W: dict[str, Any] = {}


def _faixa_de(fonte: str | Path | None, threads: int | None = None):
    if fonte is None or str(fonte) == "verdade":
        return FaixaVerdade()
    fonte = str(fonte)
    if fonte.startswith("ruido"):            # "ruido" ou "ruido:0.008" (σ de e_y na faixa, m)
        sig = float(fonte.split(":")[1]) if ":" in fonte else 0.008
        return FaixaRuidosa(sig, sig * 3.1)
    if fonte.endswith(".pt"):                 # PyTorch: usa a GPU se houver (Colab)
        import torch

        from .faixa import PreditorTorch, carregar

        dev = "cuda" if torch.cuda.is_available() else "cpu"
        modelo, tam = carregar(Path(fonte), device=dev)
        return PreditorTorch(modelo, tam, device=dev)
    return PreditorOnnx(fonte, threads=threads)


def _detector_de(fonte: str | Path, threads: int | None = None):
    from vista.placas import DetectorOnnx, DetectorVerdade

    if str(fonte) == "verdade":
        return DetectorVerdade()
    return DetectorOnnx(fonte, threads=threads)


def rodar_vista_rota(seed: int, faixa, detector, vista_cfg=None, camera: CameraParams | None = None,
                     track: Track | None = None, config: SimConfig | None = None,
                     espera_max_s: float = 10.0) -> dict[str, Any]:
    """Uma rota com o Vista completo. ``faixa``/``detector``: objetos já carregados.

    Se o Vista não largar até ``espera_max_s`` depois do verde de verdade, a
    rota acaba com ``nao_largou`` (o detector não viu o semáforo)."""
    from vista.piloto import VistaConfig, VistaPiloto
    from vista.placas import DetectorVerdade

    cfg = config or SimConfig(randomize_fx=True)
    env = VistaSimEnv(track=track, camera=camera, config=cfg, seed=seed)
    obs, info = env.reset(seed=seed)
    tr, tel = info["truth"], info["telemetry"]
    vista = VistaPiloto(env.track, env.car_params, env.cam_params, faixa, detector, vista_cfg or VistaConfig())
    vista.reset()
    ey: list[float] = []
    erros: list[tuple[float, float]] = []
    t0, n = time.perf_counter(), 0
    while True:
        if isinstance(faixa, FaixaVerdade):
            faixa.tr = tr
            if isinstance(faixa, FaixaRuidosa) and n == 0:
                faixa.rng = np.random.default_rng(seed + 99)
        if isinstance(detector, DetectorVerdade):
            detector.objetos = tr.objects
        cmd = vista.act(obs, tel)
        if vista.estado == "faixa" and vista.percepcao is not None and tr.edge is not None \
                and not isinstance(faixa, FaixaVerdade):
            erros.append((vista.percepcao.e_y - tr.e_y, math.remainder(vista.percepcao.e_psi - tr.e_psi, 2 * math.pi)))
        obs, tel, tr, done, inf = env.step(*cmd)
        n += 1
        if vista.estado != "aguardando_verde":
            ey.append(tr.e_y)
        elif env.t > env.green_at + espera_max_s:
            # o detector nunca viu o verde: na pista seria largar e ficar parado
            done, inf = True, {"reason": "nao_largou", "events": []}
        if done:
            break

    ev = Counter(e["tipo"] for e in env.events)
    juncs = env.route.junctions
    lidas = vista.leituras
    ok_manobra = sum(1 for le, j in zip(lidas, juncs, strict=False) if le.manobra == j.maneuver)
    ok_placa = sum(1 for le, j in zip(lidas, juncs, strict=False) if le.placa == j.sign)
    largada = next((e["t"] for e in vista.log if e["tipo"] == "largada"), None)
    r = {
        "semente": seed, "motivo": inf["reason"], "rota_m": round(env.route.length, 1),
        "progresso": round(min(tr.route_s / env.route.length, 1.0), 3),
        "cruzamentos": len(juncs), "cruzamentos_ok": ev.get("placa_respeitada", 0) + ev.get("cruzamento_sem_placa", 0),
        "decisoes": len(lidas), "manobras_certas": ok_manobra, "placas_certas": ok_placa,
        "reservas": sum(le.alternativa for le in lidas),
        "largou_antes_do_verde": ev.get("largou_antes_do_verde", 0) > 0,
        "reacao_verde_s": round(largada - env.green_at, 2) if largada is not None else None,
        "e_y_rms_cm": round(float(np.sqrt(np.mean(np.square(ey)))) * 100, 2) if ey else None,
        "e_y_max_cm": round(float(np.max(np.abs(ey))) * 100, 2) if ey else None,
        "contramao": ev.get("entrou_na_contramao", 0),
        "tempo_s": round(env.t, 1), "passos_s": round(n / (time.perf_counter() - t0), 1),
        "leituras": [{"no": le.no, "placa": le.placa, "certa": j.sign, "manobra": le.manobra,
                      "esperada": j.maneuver, "votos": {k: round(v, 2) for k, v in le.votos.items()},
                      "reserva": le.alternativa}
                     for le, j in zip(lidas, juncs, strict=False)],
    }
    if inf["reason"] == "desrespeitou_placa":
        r["falha"] = next((e for e in env.events if e["tipo"] == "desrespeitou_placa"), None)
    if erros:
        e = np.array(erros)
        r["erro_rede_e_y_cm"] = round(float(np.mean(np.abs(e[:, 0]))) * 100, 2)
        r["erro_rede_e_psi_graus"] = round(math.degrees(float(np.mean(np.abs(e[:, 1])))), 2)
    return r


def _init_vista(faixa, detector, vista_cfg, config, camera=None) -> None:
    cv2.setNumThreads(1)
    from vista_sim.config import repo_root

    _W["track"] = Track.load(repo_root() / "maps" / "robocar_master.yaml")
    _W["faixa"] = _faixa_de(faixa, threads=1)
    _W["detector"] = _detector_de(detector, threads=1)
    _W["vista_cfg"], _W["config"], _W["camera"] = vista_cfg, config, camera


def _rota_worker(seed: int) -> dict[str, Any]:
    return rodar_vista_rota(seed, _W["faixa"], _W["detector"], _W["vista_cfg"], camera=_W["camera"],
                            track=_W["track"], config=_W["config"])


def resumir(res: list[dict[str, Any]]) -> dict[str, Any]:
    completas = sum(r["motivo"] == "rota_completa" for r in res)
    dec = sum(r["decisoes"] for r in res)
    return {
        "rotas": len(res), "completas": completas, "taxa_conclusao": round(completas / max(len(res), 1), 3),
        "progresso_medio": round(float(np.mean([r["progresso"] for r in res])), 3) if res else 0.0,
        "motivos": dict(Counter(r["motivo"] for r in res)),
        "decisoes": dec,
        "manobras_certas": sum(r["manobras_certas"] for r in res),
        "placas_certas": sum(r["placas_certas"] for r in res),
        "reservas": sum(r["reservas"] for r in res),
        "largadas_antes_do_verde": sum(r["largou_antes_do_verde"] for r in res),
        "e_y_rms_cm_mediana": float(np.median([r["e_y_rms_cm"] for r in res if r["e_y_rms_cm"] is not None] or [np.nan])),
        "por_rota": sorted(res, key=lambda r: r["semente"]),
    }


def rodar_vista(sementes: list[int], faixa: str | Path | None = None, detector: str | Path = "verdade",
                v_cruzeiro: float = 0.5, workers: int = 1, vista_cfg=None, config: SimConfig | None = None,
                pitch_graus: float | None = None, log: Callable[[str], Any] = print) -> dict[str, Any]:
    """Avalia o Vista completo em várias rotas.

    Args:
        faixa: ``None``/"verdade" (rede perfeita), ``.onnx`` ou ``.pt`` da rede de faixa.
        detector: "verdade" (detector perfeito) ou o ``.onnx`` do YOLO.
        workers: processos em paralelo (cada um carrega os modelos, 1 thread).
        pitch_graus: experimento com outra montagem da câmera (a rede precisa ter
            sido treinada com ela).
    """
    from vista.piloto import VistaConfig

    vcfg = vista_cfg or VistaConfig()
    vcfg.v_cruzeiro = v_cruzeiro
    cfg = config or SimConfig(randomize_fx=True)
    camera = None
    if pitch_graus is not None:
        from vista_sim.config import load_camera

        camera = load_camera().with_pitch(pitch_graus)
    res: list[dict[str, Any]] = []

    def _log(r: dict[str, Any]) -> None:
        log(f"semente {r['semente']}: {r['motivo']:<18} progresso {r['progresso'] * 100:5.1f}%  "
            f"cruzamentos {r['cruzamentos_ok']}/{r['cruzamentos']}  placas {r['placas_certas']}/{r['decisoes']}  "
            f"e_y rms {r['e_y_rms_cm']} cm  ({r['passos_s']} quadros/s)")

    if workers <= 1:
        _init_vista(faixa, detector, vcfg, cfg, camera)
        for s in sementes:
            res.append(_rota_worker(s))
            _log(res[-1])
    else:
        import multiprocessing as mp

        with mp.get_context("spawn").Pool(workers, initializer=_init_vista,
                                          initargs=(None if faixa is None else str(faixa), str(detector), vcfg, cfg,
                                                    camera)) as pool:
            for r in pool.imap_unordered(_rota_worker, sementes):
                res.append(r)
                _log(r)
    out = resumir(res)
    out.update({"faixa": str(faixa) if faixa else "verdade", "detector": str(detector), "v_cruzeiro": v_cruzeiro,
                "pitch_graus": pitch_graus})
    return out
