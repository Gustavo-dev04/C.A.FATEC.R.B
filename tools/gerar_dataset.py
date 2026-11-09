"""Gera o dataset de treino: o oráculo dirige rotas sorteadas e o simulador grava.

    python -m tools.gerar_dataset --out dataset --episodios 150 --workers 8

Saída (em ``--out``):

* ``ep_SSSSSS/QQQQQ.jpg`` — imagens da câmera na resolução de processamento;
* ``rotulos.csv``          — uma linha por imagem (colunas em ``treino.rotulos.COLUNAS``);
* ``objetos.jsonl``        — placas e semáforo visíveis em cada imagem (caixa, classe,
  fração visível) — vira o dataset do detector em ``treino.deteccao``;
* ``meta.json``            — classes, resolução, câmera, parâmetros da geração.

Cada episódio é uma semente: rota, placas, cidade, luz, carro e IMU sorteados.
Além disso, por episódio:

* **ruído do oráculo** (dados de RECUPERAÇÃO): em parte dos episódios ele sai do
  centro de propósito e volta — sem isso a rede só vê o carro centrado;
* **montagem da câmera**: pitch ±2°, roll e yaw ±1° (o pitch real ainda não foi
  medido; a rede não pode depender do valor exato);
* **pós-processamento**: brilho, ruído, vinheta e desfoque de movimento;
* **velocidade**: cruzeiro do oráculo sorteado em ``--velocidades`` (padrão
  0,35–0,80 m/s) e conversão em 0,24–0,34 m/s — o Vista começa a 0,5 e vai
  subir; a rede precisa ter visto o carro (e os desvios dele) em várias
  velocidades;
* **vistas deslocadas**: para cada quadro salvo, o mesmo instante renderizado
  com o carro até ±15 cm para o lado e ±15° torto, com os rótulos recalculados.
  Cobre o "carro fora do centro" sem precisar dirigir torto (coluna ``vista``).

Salva 1 quadro a cada ``--passo`` (quadros vizinhos a 30 Hz são quase iguais);
os outros nem são renderizados.

**DAgger** (``--piloto-onnx faixa.onnx``): o VISTA dirige (``vista.piloto``:
rede de faixa + filtro na faixa, curva do mapa por odometria no cruzamento;
placas pelo detector perfeito, para o foco ser a rede de faixa) e o oráculo só
assume quando o carro passa de ``--assume-m`` de erro lateral (3,5 cm do lado do
meio-fio) ou 25° de rumo,
devolvendo quando ele estabiliza. Assim entram no treino as situações a que o
Vista SE leva — e que o oráculo dirigindo sozinho nunca visitaria. Coluna
``piloto``: quem dirigia.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import math
import os
import subprocess
import time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

from treino.rotulos import CLASSES_DETECCAO, COLUNAS, DIST_MAX, KAPPA_M, MANOBRAS
from vista_sim.config import load_camera, repo_root
from vista_sim.env import SimConfig, VistaSimEnv
from vista_sim.oracle import OracleConfig, OraclePilot
from vista_sim.track import SIGN_CLASSES, Track

_TRACK: Track | None = None
_REDES: dict[str, object] = {}


def _init_worker() -> None:
    global _TRACK
    cv2.setNumThreads(1)                       # 1 processo por núcleo; sem disputa
    _TRACK = Track.load(repo_root() / "maps" / "robocar_master.yaml")
    _ = _TRACK.texture


def episodio(args: dict) -> dict:
    """Roda um episódio e grava imagens + rótulos em ``out/ep_SSSSSS``."""
    seed, out, passo = args["seed"], Path(args["out"]), args["passo"]
    rng = np.random.default_rng(seed + 7919)   # sorteios da GERAÇÃO (o cenário usa ``seed``)
    base = load_camera(repo_root())
    if args.get("pitch_graus") is not None:
        base = base.with_pitch(args["pitch_graus"])      # experimento: outra montagem da câmera
    if args.get("res"):
        base = base.scaled(*args["res"])
    j = args["jitter_deg"]
    cam = dataclasses.replace(
        base,
        pitch=base.pitch + math.radians(rng.uniform(-j, j)),
        roll=base.roll + math.radians(rng.uniform(-j / 2, j / 2)),
        yaw=base.yaw + math.radians(rng.uniform(-j / 2, j / 2)),
    )
    dagger = bool(args.get("piloto_onnx"))
    # No DAgger quem "erra de propósito" é a própria rede: sem ruído no oráculo.
    noise = 0.0 if dagger else (float(rng.uniform(0.02, args["ruido_max"])) if rng.random() < args["ruido_prob"] else 0.0)
    env = VistaSimEnv(track=_TRACK, camera=cam, config=SimConfig(randomize_fx=True), seed=seed)
    obs, info = env.reset(seed=seed)
    env.renderer.fx.motion_blur_px = int(rng.choice([0, 0, 0, 2, 3]))
    vel = args.get("velocidades")
    v_cruz = float(rng.uniform(*vel)) if vel else 0.50
    v_curva = float(rng.uniform(0.24, min(0.34, v_cruz))) if vel else 0.28
    pilot = OraclePilot(env.track.network, info["scenario"].signs, env.car_params,
                        OracleConfig(noise=noise, v_cruise=v_cruz, v_turn=v_curva),
                        rng=np.random.default_rng(seed + 31))
    tr, tel = info["truth"], info["telemetry"]
    pilot.reset(tr.x, tr.y, tr.yaw)
    rede = None
    if dagger:
        from treino.avaliacao import PreditorOnnx
        from vista.piloto import VistaConfig, VistaPiloto
        from vista.placas import DetectorVerdade

        caminho = args["piloto_onnx"]
        if caminho not in _REDES:
            _REDES[caminho] = PreditorOnnx(caminho, threads=1)
        det_v = DetectorVerdade()
        rede = VistaPiloto(env.track, env.car_params, env.cam_params, _REDES[caminho], det_v,  # type: ignore[arg-type]
                           VistaConfig(v_cruzeiro=v_cruz, v_curva=v_curva))
        rede.reset()
    controle, t_ok = ("rede" if dagger else "oraculo"), 0.0
    assume_m, assume_psi = args.get("assume_m", 0.06), math.radians(args.get("assume_graus", 25.0))
    # À direita fica o meio-fio: na via de 80 cm sobram 8 cm entre o carro
    # centrado e a borda (mesmo limite do ruído do oráculo). Assume antes.
    assume_dir = min(assume_m, OracleConfig().noise_max_right)

    ep_dir = out / f"ep_{seed:06d}"
    ep_dir.mkdir(parents=True, exist_ok=True)
    rows, objs = [], []
    k, reason = 0, ""
    q = [cv2.IMWRITE_JPEG_QUALITY, args["qualidade"]]
    tl_troca = 0.0

    def salvar(nome: str, img, rot: dict, objetos: list, tl: str, cmd, vista: str) -> None:
        cv2.imwrite(str(out / nome), img, q)
        dist = rot["dist"]
        rows.append({
            "arquivo": nome, "semente": seed, "quadro": k, "t": round(env.t, 3),
            "e_y": round(rot["e_y"], 5), "e_psi": round(rot["e_psi"], 5), "kappa": round(rot["kappa"], 4),
            "dist_cruzamento": round(min(max(dist, 0.0), DIST_MAX) if dist is not None else DIST_MAX, 4),
            "no_cruzamento": int(rot["dentro"]), "manobra": rot["maneuver"], "placa_proxima": rot["sign"] or "",
            "semaforo": tl, "v": round(env.car.state.v, 4), "omega": round(env._yaw_rate(), 4),
            "rpm_e": round(cmd[0], 2), "rpm_d": round(cmd[1], 2), "ruido": round(noise, 3),
            "pitch_deg": round(math.degrees(cam.pitch), 3), "vista": vista, "piloto": controle, "fim": "",
        })
        objs.append({"arquivo": nome, "objetos": [
            {"kind": o["kind"], "class": o["class"], "bbox": o["bbox"], "visible_frac": o["visible_frac"],
             "truncated": o["truncated"], "distance_m": o["distance_m"], "on_route": o["on_route"]}
            for o in objetos]})

    while True:
        go = tr.traffic_light == "verde"
        cmd = pilot.act(tr.x, tr.y, tr.yaw, env.t, go)     # o oráculo sempre calcula (mantém o plano)
        if rede is not None:
            # DAgger: a rede dirige; o oráculo assume quando ela se afasta e
            # devolve quando o carro estabiliza. As imagens dessas situações
            # — que o oráculo sozinho nunca visitaria — ganham rótulo exato.
            if controle == "rede" and (tr.e_y > assume_m or tr.e_y < -assume_dir or abs(tr.e_psi) > assume_psi):
                controle, t_ok = "oraculo", 0.0
            elif controle == "oraculo":
                ok = abs(tr.e_y) < 0.02 and abs(tr.e_psi) < math.radians(6)
                t_ok = t_ok + 1 / 30 if ok else 0.0
                if t_ok >= 0.5:
                    controle = "rede"
            det_v.objetos = tr.objects
            cmd_rede = rede.act(obs, tel)          # roda sempre: mantém mapa e estado em dia
            if controle == "rede":
                cmd = cmd_rede
        k += 1
        render = (k % passo == 0) or rede is not None    # a rede precisa ver todo quadro
        # Depois da largada o semáforo real fica verde para sempre; para o
        # detector ver os 3 estados de vários ângulos, o DESENHO é sorteado
        # (o rótulo segue o desenho; o carro obedece ao real). No DAgger não:
        # o Vista larga pelo semáforo que VÊ.
        if go and env.t >= tl_troca and not dagger:
            env.tl_override = str(rng.choice(["vermelho", "amarelo", "verde"]))
            tl_troca = env.t + float(rng.uniform(1.0, 3.0))
        obs, tel, tr, done, inf = env.step(*cmd, render=render)
        parado = tr.v < 0.02 and not go
        if k % passo == 0 and (not parado or k % (passo * 10) == 0):   # largada parada: poucas
            tl = env.tl_override or tr.traffic_light
            rot = env.rotulos_em(tr.x, tr.y, tr.yaw, tr.route_s)
            salvar(f"{ep_dir.name}/{k:05d}.jpg", obs, rot, tr.objects, tl, cmd, "real")
            # Vistas DESLOCADAS: o mesmo instante visto de um carro fora do
            # centro e torto — rótulos exatos, sem precisar dirigir torto.
            for v in range(args["vistas_extras"] if go else 0):
                for _ in range(6):
                    dy = float(rng.uniform(-args["desloc_m"], args["desloc_m"]))
                    dpsi = math.radians(float(rng.uniform(-args["desloc_graus"], args["desloc_graus"])))
                    x2 = tr.x - dy * math.sin(tr.yaw)
                    y2 = tr.y + dy * math.cos(tr.yaw)
                    rot2 = env.rotulos_em(x2, y2, tr.yaw + dpsi, tr.route_s)
                    if rot2["on_carpet"]:
                        img2, info2 = env.renderer.render((x2, y2, tr.yaw + dpsi), tl)
                        salvar(f"{ep_dir.name}/{k:05d}_d{v}.jpg", img2, rot2, info2["objects"], tl, cmd, "deslocada")
                        break
        if done or (args.get("max_quadros") and k >= args["max_quadros"]):
            reason = inf["reason"] or "corte"
            break
    if rows:
        rows[-1]["fim"] = reason
    with open(ep_dir / "rotulos.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=COLUNAS)
        w.writeheader()
        w.writerows(rows)
    with open(ep_dir / "objetos.jsonl", "w", encoding="utf-8") as fh:
        for o in objs:
            fh.write(json.dumps(o, ensure_ascii=False) + "\n")
    return {"seed": seed, "motivo": reason, "imagens": len(rows), "rota_m": round(env.route.length, 1),
            "ruido": noise, "v_cruzeiro": round(v_cruz, 3),
            "frac_rede": round(sum(r["piloto"] == "rede" for r in rows) / max(len(rows), 1), 3)}


def juntar(out: Path) -> tuple[list[int], int]:
    """Concatena os rótulos de TODOS os episódios da pasta (inclusive de
    gerações anteriores — dá para gerar em partes) num arquivo só."""
    eps = sorted(p for p in out.glob("ep_*") if p.is_dir())
    n = 0
    with open(out / "rotulos.csv", "w", newline="", encoding="utf-8") as fo:
        w = csv.DictWriter(fo, fieldnames=COLUNAS)
        w.writeheader()
        for ep in eps:
            f = ep / "rotulos.csv"
            if f.exists():
                with open(f, encoding="utf-8") as fi:
                    for row in csv.DictReader(fi):
                        w.writerow(row)
                        n += 1
    with open(out / "objetos.jsonl", "w", encoding="utf-8") as fo:
        for ep in eps:
            f = ep / "objetos.jsonl"
            if f.exists():
                fo.write(f.read_text(encoding="utf-8"))
    return [int(ep.name[3:]) for ep in eps], n


def _git_commit() -> str:
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True,
                              cwd=repo_root(), check=False).stdout.strip()
    except OSError:
        return ""


def gerar(out: Path, episodios: int, semente_inicial: int = 0, passo: int = 4, workers: int | None = None,
          res: tuple[int, int] | None = None, ruido_prob: float = 0.6, ruido_max: float = 0.04,
          jitter_deg: float = 2.0, qualidade: int = 92, max_quadros: int | None = None,
          vistas_extras: int = 1, desloc_m: float = 0.15, desloc_graus: float = 15.0,
          piloto_onnx: Path | str | None = None, assume_m: float = 0.06,
          velocidades: tuple[float, float] | None = (0.35, 0.80), pitch_graus: float | None = None,
          log=print) -> dict:
    from multiprocessing import get_context

    out.mkdir(parents=True, exist_ok=True)
    workers = workers or os.cpu_count() or 1
    jobs = [{"seed": s, "out": str(out), "passo": passo, "res": res, "ruido_prob": ruido_prob,
             "ruido_max": ruido_max, "jitter_deg": jitter_deg, "qualidade": qualidade,
             "max_quadros": max_quadros, "vistas_extras": vistas_extras, "desloc_m": desloc_m,
             "desloc_graus": desloc_graus, "piloto_onnx": str(piloto_onnx) if piloto_onnx else None,
             "assume_m": assume_m, "velocidades": velocidades, "pitch_graus": pitch_graus}
            for s in range(semente_inicial, semente_inicial + episodios)]
    t0 = time.perf_counter()
    res_eps = []
    with get_context("spawn").Pool(workers, initializer=_init_worker) as pool:
        for i, r in enumerate(pool.imap_unordered(episodio, jobs), 1):
            res_eps.append(r)
            if i % max(1, episodios // 20) == 0 or i == episodios:
                n_img = sum(x["imagens"] for x in res_eps)
                log(f"  {i}/{episodios} episódios, {n_img} imagens, {time.perf_counter() - t0:.0f} s")
    sementes, n_img = juntar(out)
    cam = load_camera(repo_root())
    if res:
        cam = cam.scaled(*res)
    meta = {
        "episodios": len(sementes), "imagens": n_img, "sementes": sementes,
        "motivos_fim_desta_geracao": dict(Counter(r["motivo"] for r in res_eps)),
        "largura": cam.width, "altura": cam.height, "passo": passo,
        "ruido_prob": ruido_prob, "ruido_max": ruido_max, "jitter_deg": jitter_deg,
        "vistas_extras": vistas_extras, "desloc_m": desloc_m, "desloc_graus": desloc_graus,
        "dagger_desta_geracao": str(piloto_onnx) if piloto_onnx else None,
        "velocidades": list(velocidades) if velocidades else None,
        "pitch_graus": pitch_graus if pitch_graus is not None else round(math.degrees(cam.pitch), 2),
        "manobras": list(MANOBRAS), "classes_placa": list(SIGN_CLASSES),
        "classes_deteccao": list(CLASSES_DETECCAO), "dist_max": DIST_MAX, "kappa_janela_m": KAPPA_M,
        "commit": _git_commit(), "segundos": round(time.perf_counter() - t0, 1),
    }
    (out / "meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
    return meta


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", default="dataset")
    ap.add_argument("--episodios", type=int, default=150)
    ap.add_argument("--semente-inicial", type=int, default=0)
    ap.add_argument("--passo", type=int, default=4, help="salva 1 quadro a cada N")
    ap.add_argument("--workers", type=int, default=None)
    ap.add_argument("--res", default=None, help="LxA, ex.: 320x240 (padrão: resolução de processamento)")
    ap.add_argument("--ruido-prob", type=float, default=0.6)
    ap.add_argument("--ruido-max", type=float, default=0.04)
    ap.add_argument("--jitter-deg", type=float, default=2.0)
    ap.add_argument("--qualidade", type=int, default=92)
    ap.add_argument("--max-quadros", type=int, default=None, help="corta cada episódio (testes)")
    ap.add_argument("--vistas-extras", type=int, default=1, help="vistas deslocadas por quadro salvo")
    ap.add_argument("--desloc-m", type=float, default=0.15)
    ap.add_argument("--desloc-graus", type=float, default=15.0)
    ap.add_argument("--piloto-onnx", default=None,
                    help="DAgger: a rede (ONNX) dirige e o oráculo assume quando ela se afasta")
    ap.add_argument("--assume-m", type=float, default=0.06, help="DAgger: |e_y| em que o oráculo assume")
    ap.add_argument("--velocidades", default="0.35:0.80",
                    help="faixa de velocidade de cruzeiro sorteada por episódio (m/s); 'fixa' = 0,5")
    ap.add_argument("--pitch-graus", type=float, default=None,
                    help="experimento: pitch da câmera (padrão: config/camera/extrinsics.yaml)")
    a = ap.parse_args(argv)
    res = tuple(int(v) for v in a.res.lower().split("x")) if a.res else None
    vel = None if a.velocidades == "fixa" else tuple(float(v) for v in a.velocidades.split(":"))
    meta = gerar(Path(a.out), a.episodios, a.semente_inicial, a.passo, a.workers, res,  # type: ignore[arg-type]
                 a.ruido_prob, a.ruido_max, a.jitter_deg, a.qualidade, a.max_quadros,
                 a.vistas_extras, a.desloc_m, a.desloc_graus, a.piloto_onnx, a.assume_m, vel,  # type: ignore[arg-type]
                 a.pitch_graus)
    print(json.dumps({k: v for k, v in meta.items() if k != "sementes"}, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
