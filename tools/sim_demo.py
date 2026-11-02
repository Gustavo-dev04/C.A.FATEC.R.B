"""Demonstração e benchmark do Vista-Sim.

    python -m tools.sim_demo --out runs/demo           # uma volta + vídeo
    python -m tools.sim_demo --bench                   # só mede passos/s

Critério de pronto do S1 (CLAUDE.md §5A): vídeo da câmera virtual dando uma
volta no anel externo; ≥ 1.000 passos/s em 320×240.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np

from vista_sim.config import load_camera, repo_root
from vista_sim.env import SimConfig, VistaSimEnv
from vista_sim.oracle import OracleConfig, OraclePilot
from vista_sim.render_fast import FastRenderer
from vista_sim.track import Track


def to_h264(path: Path) -> bool:
    """Reconverte o vídeo para H.264, que navegadores e celulares tocam.

    O OpenCV do pip só grava ``mp4v`` (MPEG-4 Part 2), que NÃO toca no
    navegador. Com ffmpeg disponível, troca no lugar; sem ele, mantém o
    original e avisa.
    """
    if shutil.which("ffmpeg") is None:
        print(f"aviso: ffmpeg não encontrado; {path.name} ficou em mp4v (não toca no navegador)")
        return False
    tmp = path.with_suffix(".h264.mp4")
    cmd = ["ffmpeg", "-y", "-v", "error", "-i", str(path), "-c:v", "libx264", "-preset", "slow",
           "-crf", "26", "-pix_fmt", "yuv420p", "-movflags", "+faststart", "-an", str(tmp)]
    if subprocess.run(cmd, check=False).returncode != 0:
        tmp.unlink(missing_ok=True)
        return False
    tmp.replace(path)
    return True


def overlay(img: np.ndarray, truth, fps: float) -> np.ndarray:
    out = cv2.resize(img, (img.shape[1] * 2, img.shape[0] * 2), interpolation=cv2.INTER_LINEAR)
    tl = {"vermelho": (40, 40, 230), "amarelo": (30, 190, 240), "verde": (60, 200, 60)}[truth.traffic_light]
    lines = [
        f"t={truth.t:6.2f}s  v={truth.v:4.2f} m/s",
        f"e_y={truth.e_y * 100:+5.1f} cm  e_psi={math.degrees(truth.e_psi):+5.1f} deg",
        f"semaforo: {truth.traffic_light}",
        f"sim: {fps:5.0f} passos/s",
    ]
    cv2.rectangle(out, (0, 0), (300, 22 * len(lines) + 8), (0, 0, 0), -1)
    for i, txt in enumerate(lines):
        cv2.putText(out, txt, (8, 22 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    tl if i == 2 else (255, 255, 255), 1, cv2.LINE_AA)
    for ob in truth.objects:
        if ob["kind"] == "placa" and min(ob["bbox_px"]) >= 6:
            x0, y0, x1, y1 = (np.array(ob["bbox"]) * 2).tolist()
            cv2.rectangle(out, (x0, y0), (x1, y1), (0, 255, 255), 1)
            cv2.putText(out, f"{ob['class']} {ob['distance_m']:.1f}m", (x0, max(12, y0 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)
    return out


def run_lap(out_dir: Path, seed: int, noise: float) -> dict:
    env = VistaSimEnv(config=SimConfig(latency_s=0.10), seed=seed)
    pilot = OraclePilot(env.lane.line, OracleConfig(noise=noise), rng=np.random.default_rng(seed))
    obs, info = env.reset(seed=seed)
    truth = info["truth"]
    lane_len = env.lane.line.length

    out_dir.mkdir(parents=True, exist_ok=True)
    video = cv2.VideoWriter(str(out_dir / "volta.mp4"), cv2.VideoWriter_fourcc(*"mp4v"),
                            env.cfg.camera_hz, (obs.shape[1] * 2, obs.shape[0] * 2))
    traveled, last_s, started = 0.0, None, False
    e_y, frames, sheet = [], 0, []
    jumped, sim_time = False, 0.0
    reason = ""
    while True:
        go = truth.traffic_light == "verde"
        if go and not started:
            started = True
            jumped = env.t < env.green_at       # nunca deve acontecer
        v, w = pilot.act(truth.x, truth.y, truth.yaw, env.t, go)
        t_step = time.perf_counter()
        obs, telem, truth, done, info = env.step(v, w)
        sim_time += time.perf_counter() - t_step   # só o simulador, sem vídeo
        frames += 1
        if started:
            if last_s is not None:
                ds = truth.s - last_s
                if ds < -lane_len / 2:
                    ds += lane_len
                traveled += max(ds, 0.0)
            last_s = truth.s
            e_y.append(truth.e_y)
        fps = frames / max(sim_time, 1e-6)
        frame = overlay(obs, truth, fps)
        video.write(frame)
        if frames % 30 == 0:
            sheet.append(cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA))
        if done:
            reason = info["reason"]
            break
        if traveled >= lane_len:
            reason = "volta_completa"
            break
    video.release()
    to_h264(out_dir / "volta.mp4")

    if sheet:
        # 12 quadros espalhados pela volta inteira, não só a largada
        pick = [sheet[i] for i in np.linspace(0, len(sheet) - 1, 12).round().astype(int)]
        rows = [np.hstack(pick[i:i + 4]) for i in range(0, 12, 4)]
        cv2.imwrite(str(out_dir / "quadros.jpg"), np.vstack(rows))

    e = np.array(e_y) if e_y else np.zeros(1)
    result = {
        "motivo_fim": reason,
        "volta_completa": reason == "volta_completa",
        "largou_antes_do_verde": jumped,
        "tempo_sim_s": round(env.t, 2),
        "verde_em_s": round(env.green_at, 2),
        "tempo_de_volta_s": round(env.t - env.green_at, 2),
        "comprimento_da_faixa_m": round(lane_len, 3),
        "e_y_rms_cm": round(float(np.sqrt(np.mean(e**2))) * 100, 2),
        "e_y_max_cm": round(float(np.max(np.abs(e))) * 100, 2),
        "quadros": frames,
        "passos_por_s_simulador": round(frames / max(sim_time, 1e-6), 1),
        "tempo_real_x": round(env.t / max(sim_time, 1e-6), 1),
    }
    (out_dir / "resultado.json").write_text(json.dumps(result, indent=2, ensure_ascii=False))
    return result


def bench(n: int = 600) -> dict:
    root = repo_root()
    track = Track.load(root / "maps" / "robocar_master.yaml")
    _ = track.texture
    cam_full = load_camera(root)
    res = {}
    for name, (w, h) in {"320x240": (320, 240), "processamento": (cam_full.width, cam_full.height),
                         "1640x1232": (1640, 1232)}.items():
        cam = cam_full.scaled(w, h)
        r = FastRenderer(track, cam)
        lane = track.lanes["anel_antihorario"].line
        poses = [lane.point_at(i * lane.length / n) for i in range(n)]
        reps = n if w * h <= 200_000 else n // 10
        t0 = time.perf_counter()
        for i in range(reps):
            r.render(poses[i % n], "verde")
        dt = time.perf_counter() - t0
        res[name] = {"resolucao": f"{w}x{h}", "passos_por_s": round(reps / dt, 1)}
    return res


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="runs/demo")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--noise", type=float, default=0.0, help="desvio lateral do oráculo (m)")
    ap.add_argument("--bench", action="store_true")
    args = ap.parse_args(argv)
    if args.bench:
        print(json.dumps(bench(), indent=2, ensure_ascii=False))
        return 0
    print(json.dumps(run_lap(Path(args.out), args.seed, args.noise), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
