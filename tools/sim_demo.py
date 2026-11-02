"""Demonstração e benchmark do Vista-Sim.

    python -m tools.sim_demo --out runs/demo --seed 139 # rota pela cidade + vídeo
    python -m tools.sim_demo --bench                    # mede passos/s

Gera em ``--out``:

* ``rota.mp4``      — câmera virtual (esq.) + mapa visto de cima (dir.), H.264;
* ``quadros.jpg``   — 12 quadros espalhados pela rota;
* ``mapa_rota.png`` — rota, placas (da rota e distratoras), zebras, prédios, luz;
* ``iluminacao.jpg``— o MESMO ponto da pista sob 6 iluminações sorteadas;
* ``resultado.json``— motivo do fim, cada cruzamento (placa → manobra), erros.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

from vista_sim.config import load_camera, repo_root
from vista_sim.env import SimConfig, VistaSimEnv
from vista_sim.oracle import OracleConfig, OraclePilot
from vista_sim.scenario import generate
from vista_sim.track import Track

SHORT = {
    "seguir_em_frente": "SIGA EM FRENTE", "virar_esquerda": "VIRE A ESQUERDA",
    "virar_direita": "VIRE A DIREITA", "proibido_seguir_em_frente": "PROIBIDO FRENTE",
    "proibido_virar_esquerda": "PROIBIDO ESQUERDA", "proibido_virar_direita": "PROIBIDO DIREITA",
}


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
           "-crf", "28", "-pix_fmt", "yuv420p", "-movflags", "+faststart", "-an", str(tmp)]
    if subprocess.run(cmd, check=False).returncode != 0:
        tmp.unlink(missing_ok=True)
        return False
    tmp.replace(path)
    return True


# --------------------------------------------------------------------------
# mapa visto de cima
# --------------------------------------------------------------------------


class MiniMap:
    """Textura do chão (com luz e zebras) reduzida + rota, placas e prédios."""

    def __init__(self, env: VistaSimEnv, height: int) -> None:
        t, r = env.track, env.renderer
        tex = r._tex
        k = height / tex.shape[0]
        self.k_px = k / t.res                         # px do mapa por metro
        self.track = t
        base = cv2.resize(tex, (int(tex.shape[1] * k), height), interpolation=cv2.INTER_AREA)
        sc = env.scenario
        for b in sc.buildings:
            cv2.fillPoly(base, [self.px(b["footprint"])], tuple(int(c) for c in b["color"]), cv2.LINE_AA)
            cv2.polylines(base, [self.px(b["footprint"])], True, (30, 30, 30), 1, cv2.LINE_AA)
        for p in sc.people:
            cv2.circle(base, tuple(self.px([[p["x"], p["y"]]])[0]), 4, tuple(int(c) for c in p["shirt"]), -1)
        cv2.polylines(base, [self.px(env.route.points)], False, (0, 200, 255), 2, cv2.LINE_AA)
        for s in sc.signs:
            c = tuple(self.px([[s["x"], s["y"]]])[0])
            col = (40, 40, 230) if s["on_route"] else (150, 150, 150)
            cv2.circle(base, c, 6 if s["on_route"] else 4, col, -1, cv2.LINE_AA)
        for lamp in sc.lighting["lamps"]:
            c = self.px([[lamp["x"], lamp["y"]]])[0]
            if 0 <= c[0] < base.shape[1] and 0 <= c[1] < base.shape[0]:
                cv2.drawMarker(base, tuple(c), (0, 240, 255), cv2.MARKER_STAR, 14, 2)
        tl = t.traffic_light
        if tl:
            cv2.rectangle(base, tuple(self.px([[tl["x"] - 0.08, tl["y"] - 0.08]])[0]),
                          tuple(self.px([[tl["x"] + 0.08, tl["y"] + 0.08]])[0]), (20, 20, 20), -1)
        self.base = base
        self.trail: list[tuple[int, int]] = []

    def px(self, pts) -> np.ndarray:
        p = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
        u = (p[:, 0] - self.track.x_min) * self.k_px
        v = (self.track.y_max - p[:, 1]) * self.k_px
        return np.round(np.column_stack([u, v])).astype(np.int32)

    def draw(self, x: float, y: float, yaw: float, foot: np.ndarray) -> np.ndarray:
        img = self.base.copy()
        c = tuple(self.px([[x, y]])[0])
        if not self.trail or self.trail[-1] != c:
            self.trail.append(c)
        if len(self.trail) > 1:
            cv2.polylines(img, [np.array(self.trail, np.int32)], False, (60, 255, 60), 2, cv2.LINE_AA)
        cv2.fillPoly(img, [self.px(foot)], (255, 255, 255), cv2.LINE_AA)
        tip = (int(c[0] + 14 * math.cos(yaw)), int(c[1] - 14 * math.sin(yaw)))
        cv2.arrowedLine(img, c, tip, (0, 0, 0), 2, cv2.LINE_AA, tipLength=0.4)
        return img


def _txt(img, s, org, scale=0.55, col=(255, 255, 255), th=1):
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), th + 1, cv2.LINE_AA)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, col, th, cv2.LINE_AA)


def overlay(img: np.ndarray, truth, telem, cmd: tuple[float, float], fps: float,
            banner: tuple[str, tuple[int, int, int]] | None) -> np.ndarray:
    out = cv2.resize(img, (img.shape[1] * 2, img.shape[0] * 2), interpolation=cv2.INTER_LINEAR)
    for ob in truth.objects:
        if ob["kind"] == "placa" and min(ob["bbox_px"]) >= 5:
            x0, y0, x1, y1 = (np.array(ob["bbox"]) * 2).tolist()
            col = (0, 255, 255) if ob["on_route"] else (170, 170, 170)
            cv2.rectangle(out, (x0, y0), (x1, y1), col, 1)
            vis = "" if ob["visible_frac"] > 0.98 else f" vis {ob['visible_frac'] * 100:.0f}%"
            _txt(out, f"{SHORT[ob['class']]} {ob['distance_m']:.1f}m{vis}", (x0, max(12, y0 - 4)), 0.4, col)
    tl = {"vermelho": (40, 40, 230), "amarelo": (30, 190, 240), "verde": (60, 200, 60)}[truth.traffic_light]
    lines = [
        (f"t={truth.t:6.2f}s  v={truth.v:4.2f} m/s", (255, 255, 255)),
        (f"rpm alvo  E {cmd[0]:5.0f}  D {cmd[1]:5.0f}", (255, 255, 255)),
        (f"rpm real  E {truth.rpm_left:5.0f}  D {truth.rpm_right:5.0f}", (255, 255, 255)),
        (f"encoder   E {telem.rpm_left:5.0f}  D {telem.rpm_right:5.0f}", (200, 200, 200)),
        (f"giro z {math.degrees(telem.gyro_z):+7.1f} deg/s  vies {math.degrees(telem.gyro_bias_est):+6.2f}"
         + ("" if telem.imu_calibrated else " (calibrando)"), (200, 200, 200)),
        (f"rumo odo {math.degrees(math.remainder(telem.psi - truth.yaw, 2 * math.pi)):+5.1f} deg do real",
         (200, 200, 200)),
        (f"e_y={truth.e_y * 100:+5.1f} cm  e_psi={math.degrees(truth.e_psi):+5.1f} deg", (255, 255, 255)),
        (f"semaforo: {truth.traffic_light}", tl),
    ]
    nj = truth.next_junction
    if nj is not None:
        sg = SHORT.get(nj["sign"], "sem placa") if nj["sign"] else "sem placa"
        lines.append((f"prox. cruzamento {nj['node']} em {max(nj['distance_m'], 0):.1f} m", (0, 255, 255)))
        lines.append((f"  {sg} -> {nj['maneuver'].upper()}", (0, 255, 255)))
    lines.append((f"sim: {fps:4.0f} passos/s", (180, 180, 180)))
    cv2.rectangle(out, (0, 0), (330, 21 * len(lines) + 8), (0, 0, 0), -1)
    for i, (s, col) in enumerate(lines):
        cv2.putText(out, s, (8, 20 + 21 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
    if banner:
        s, col = banner
        (w, h), _ = cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, 0.75, 2)
        x0 = (out.shape[1] - w) // 2
        cv2.rectangle(out, (x0 - 10, out.shape[0] - h - 26), (x0 + w + 10, out.shape[0] - 8), (0, 0, 0), -1)
        cv2.putText(out, s, (x0, out.shape[0] - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.75, col, 2, cv2.LINE_AA)
    return out


# --------------------------------------------------------------------------
# rota completa
# --------------------------------------------------------------------------


def run_route(out_dir: Path, seed: int, noise: float, video: bool = True) -> dict:
    env = VistaSimEnv(config=SimConfig(latency_s=0.10), seed=seed)
    obs, info = env.reset(seed=seed)
    sc = info["scenario"]
    truth, telem = info["truth"], info["telemetry"]
    pilot = OraclePilot(env.track.network, sc.signs, env.car_params, OracleConfig(noise=noise),
                        rng=np.random.default_rng(seed))
    pilot.reset(truth.x, truth.y, truth.yaw)

    out_dir.mkdir(parents=True, exist_ok=True)
    mm = MiniMap(env, obs.shape[0] * 2)
    if video:
        H = obs.shape[0] * 2
        W = obs.shape[1] * 2 + mm.base.shape[1]
        vw = cv2.VideoWriter(str(out_dir / "rota.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), env.cfg.camera_hz, (W, H))
    cv2.imwrite(str(out_dir / "mapa_rota.png"), mm.base)

    frames, sim_time, sheet = 0, 0.0, []
    e_y, cmd, banner, banner_until = [], (0.0, 0.0), None, -1.0
    reason = ""
    while True:
        go = truth.traffic_light == "verde"
        cmd = pilot.act(truth.x, truth.y, truth.yaw, env.t, go)
        t0 = time.perf_counter()
        obs, telem, truth, done, info = env.step(*cmd)
        sim_time += time.perf_counter() - t0                 # só o simulador
        frames += 1
        if go:
            e_y.append(truth.e_y)
        for ev in info["events"]:
            if ev["tipo"] == "placa_respeitada":
                banner = (f"{ev['no']}: {SHORT[ev['placa']]} -> {ev['manobra'].upper()}  OK", (60, 230, 60))
            elif ev["tipo"] == "cruzamento_sem_placa":
                banner = (f"{ev['no']}: sem placa -> FRENTE  OK", (200, 230, 200))
            elif ev["tipo"] in ("desrespeitou_placa", "saiu_da_pista", "saiu_da_rota"):
                banner = (ev["tipo"].upper().replace("_", " "), (40, 40, 255))
            elif ev["tipo"] == "rota_completa":
                banner = ("ROTA COMPLETA", (60, 230, 60))
            banner_until = env.t + 2.5
        fps = frames / max(sim_time, 1e-6)
        cam = overlay(obs, truth, telem, cmd, fps, banner if env.t < banner_until else None)
        if video:
            vw.write(np.hstack([cam, mm.draw(truth.x, truth.y, truth.yaw, env.car.footprint())]))
        if frames % 15 == 0:
            sheet.append(cv2.resize(cam, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA))
        if done:
            reason = info["reason"]
            break
    if video:
        for _ in range(int(env.cfg.camera_hz * 1.5)):             # segura o último quadro
            vw.write(np.hstack([cam, mm.draw(truth.x, truth.y, truth.yaw, env.car.footprint())]))
        vw.release()
        to_h264(out_dir / "rota.mp4")
    if sheet:
        pick = [sheet[i] for i in np.linspace(0, len(sheet) - 1, 12).round().astype(int)]
        cv2.imwrite(str(out_dir / "quadros.jpg"), np.vstack([np.hstack(pick[i:i + 4]) for i in range(0, 12, 4)]))

    e = np.array(e_y) if e_y else np.zeros(1)
    evs = env.events
    cruz = [{"no": ev["no"], "placa": ev.get("placa"), "manobra": ev.get("manobra") or ev.get("feito"),
             "ok": ev["tipo"] != "desrespeitou_placa"}
            for ev in evs if ev["tipo"] in ("placa_respeitada", "cruzamento_sem_placa", "desrespeitou_placa")]
    result = {
        "semente": seed,
        "motivo_fim": reason,
        "rota_completa": reason == "rota_completa",
        "entrou_no_centro": any(c["no"].startswith("C_") for c in cruz),
        "cruzamentos": cruz,
        "placas_na_rota": sum(1 for s in sc.signs if s["on_route"]),
        "placas_distratoras": sum(1 for s in sc.signs if not s["on_route"]),
        "classes_de_placa_na_rota": dict(Counter(s["class"] for s in sc.signs if s["on_route"])),
        "faixas_de_pedestre_listras": len(sc.crosswalks),
        "predios": len(sc.buildings),
        "pessoas": len(sc.people),
        "lampadas": len(sc.lighting["lamps"]),
        "luz_ambiente": round(sc.lighting["ambient"], 2),
        "exposicao": round(sc.lighting["exposure"], 2),
        "eventos": dict(Counter(ev["tipo"] for ev in evs)),
        "comprimento_da_rota_m": round(env.route.length, 2),
        "tempo_sim_s": round(env.t, 2),
        "verde_em_s": round(env.green_at, 2),
        "tempo_de_prova_s": round(env.t - env.green_at, 2),
        "e_y_rms_cm": round(float(np.sqrt(np.mean(e**2))) * 100, 2),
        "e_y_max_cm": round(float(np.max(np.abs(e))) * 100, 2),
        "erro_odometria_fim_m": {
            "encoders_e_giroscopio": round(math.hypot(telem.x - truth.x, telem.y - truth.y), 3),
            "so_encoders": round(math.hypot(env.odometry.enc_pose[0] - truth.x,
                                            env.odometry.enc_pose[1] - truth.y), 3),
        },
        "mpu6050": {
            "vies_real_dps": round(math.degrees(truth.gyro_bias), 3),
            "vies_estimado_no_vermelho_dps": round(math.degrees(telem.gyro_bias_est), 3),
            "fator_de_escala": round(env.imu.gyro_scale, 4),
        },
        "rodas": {"raio_esq_mm": round(env.car.state.left.radius * 1000, 3),
                  "raio_dir_mm": round(env.car.state.right.radius * 1000, 3),
                  "patinacao": round(env.car.state.left.slip, 4)},
        "quadros": frames,
        "passos_por_s_simulador": round(frames / max(sim_time, 1e-6), 1),
        "tempo_real_x": round(env.t / max(sim_time, 1e-6), 1),
    }
    (out_dir / "resultado.json").write_text(json.dumps(result, indent=2, ensure_ascii=False))
    lighting_sheet(out_dir / "iluminacao.jpg", env, seed)
    return result


def lighting_sheet(path: Path, env: VistaSimEnv, seed: int, n: int = 6) -> None:
    """O mesmo ponto da pista (com prédios e público) sob iluminações diferentes."""
    sc = env.scenario
    j = next((j for j in env.route.junctions if j.node.startswith("C_")), env.route.junctions[0])
    x, y, h = env.route.line.point_at(max(j.s_in - 1.4, 0.0))
    tiles = []
    for k in range(n):
        other = generate(env.track.data, env.track.network, seed * 1000 + k + 1)
        sc_k = type(sc)(**{**sc.__dict__, "lighting": other.lighting})
        env.renderer.set_scenario(sc_k, lighting=True)
        img, _ = env.renderer.render((x, y, h), "verde")
        L = other.lighting
        _txt(img, f"{len(L['lamps'])} lamp. amb {L['ambient']:.2f} exp {L['exposure']:.2f}", (6, 16), 0.45)
        tiles.append(img)
    env.renderer.set_scenario(sc, lighting=True)
    cv2.imwrite(str(path), np.vstack([np.hstack(tiles[i:i + 3]) for i in range(0, n, 3)]))


# --------------------------------------------------------------------------
# benchmark
# --------------------------------------------------------------------------


def bench(n: int = 400) -> dict:
    """passos/s em três níveis: só chão; cidade inteira (prédios, público,
    placas, luz e sombra); e o ``env.step`` completo (física + verdade)."""
    root = repo_root()
    track = Track.load(root / "maps" / "robocar_master.yaml")
    _ = track.texture
    cam_full = load_camera(root)
    res: dict[str, dict] = {}
    for name, (w, h) in {"320x240": (320, 240), "processamento": (cam_full.width, cam_full.height)}.items():
        cam = cam_full.scaled(w, h)
        env = VistaSimEnv(track=track, camera=cam, seed=0)
        env.reset(seed=7)
        P = env.route.points[::25]
        poses = [(p[0], p[1], math.atan2(*(b - p)[::-1])) for p, b in zip(P[:-1], P[1:], strict=True)]
        r = env.renderer
        row = {"resolucao": f"{w}x{h}"}
        # cidade inteira, poses em sequência pela rota (como dirigindo)
        t0 = time.perf_counter()
        for i in range(n):
            r.render(poses[i % len(poses)], "verde")
        row["cidade_com_luz"] = round(n / (time.perf_counter() - t0), 1)
        r.set_scenario(None)
        t0 = time.perf_counter()
        for i in range(n):
            r.render(poses[i % len(poses)], "verde")
        row["so_chao"] = round(n / (time.perf_counter() - t0), 1)
        # env.step completo com o oráculo
        obs, info = env.reset(seed=7)
        env._tl_yellow_at = env._tl_green_at = 0.0
        pilot = OraclePilot(env.track.network, info["scenario"].signs, env.car_params)
        tr = info["truth"]
        pilot.reset(tr.x, tr.y, tr.yaw)
        t_env = 0.0
        for _ in range(n):
            cmd = pilot.act(tr.x, tr.y, tr.yaw, env.t, True)
            t0 = time.perf_counter()
            _, _, tr, done, _ = env.step(*cmd)
            t_env += time.perf_counter() - t0
            if done:
                break
        row["env_step_completo"] = round(n / t_env, 1)
        res[name] = row
    return res


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="runs/demo")
    ap.add_argument("--seed", type=int, default=139)
    ap.add_argument("--noise", type=float, default=0.0, help="desvio lateral do oráculo (m)")
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--bench", action="store_true")
    args = ap.parse_args(argv)
    if args.bench:
        print(json.dumps(bench(), indent=2, ensure_ascii=False))
        return 0
    print(json.dumps(run_route(Path(args.out), args.seed, args.noise, not args.no_video), indent=2,
                     ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
