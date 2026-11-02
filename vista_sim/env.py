"""Ambiente do Vista-Sim — API no estilo ``gymnasium``.

    env = VistaSimEnv()
    obs, info = env.reset(seed=0)
    while True:
        obs, telem, truth, done, info = env.step(v=0.5, omega=0.0)

O que o Vista recebe (``obs`` + ``telem``) é exatamente o que receberia do carro
real: a imagem da câmera e a telemetria do ESP32 — odometria INTEGRADA DOS
ENCODERS, não a pose verdadeira. A pose verdadeira só aparece em ``truth``,
que serve para medir o Vista e rotular dados; o Vista nunca deve lê-la.

Cada ``step`` avança um quadro da câmera (30 Hz). Dentro dele, o carro é
integrado em subpassos e os comandos chegam com atraso configurável
(exposição + inferência + serial; CLAUDE.md §5A sugere 80–120 ms).
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from .car import DiffDriveCar
from .config import CameraParams, CarParams, load_camera, load_car, repo_root
from .geometry import wrap_angle
from .render_fast import FastRenderer, PostFx
from .track import Track


@dataclass
class SimConfig:
    camera_hz: float = 30.0
    physics_dt: float = 0.005
    latency_s: float = 0.10
    slip: float = 0.0
    lane: str = "anel_antihorario"
    # Semáforo: vermelho → amarelo → verde (regulamento 3.6). Tempos sorteados
    # no reset para o Vista não decorar um instante fixo.
    tl_red_s: tuple[float, float] = (2.0, 6.0)
    tl_yellow_s: tuple[float, float] = (1.0, 3.0)
    randomize_fx: bool = False
    max_episode_s: float = 180.0


@dataclass
class Telemetry:
    """Espelha o quadro ``T,...`` do ESP32 (CLAUDE.md §4.1)."""

    seq: int
    t: float
    x: float                  # odometria (m) — integrada dos encoders
    y: float
    psi: float                # rad
    v_left: float             # m/s, medida pelos encoders
    v_right: float
    ticks_left: int
    ticks_right: int


@dataclass
class Truth:
    """Verdade de campo exportada em todo quadro (CLAUDE.md §5A)."""

    t: float
    x: float
    y: float
    yaw: float
    v: float
    omega: float
    lane: str
    s: float
    e_y: float                # m, + = à esquerda do centro da faixa
    e_psi: float              # rad, + = apontando para a esquerda da faixa
    curvature: float
    on_carpet: bool
    in_own_lane: bool
    traffic_light: str
    objects: list[dict[str, Any]] = field(default_factory=list)


class VistaSimEnv:
    def __init__(
        self,
        track: Track | None = None,
        car: CarParams | None = None,
        camera: CameraParams | None = None,
        config: SimConfig | None = None,
        map_path: Path | str | None = None,
        seed: int | None = None,
    ) -> None:
        root = repo_root()
        self.track = track or Track.load(map_path or root / "maps" / "robocar_master.yaml")
        self.car_params = car or load_car(root)
        self.cam_params = camera or load_camera(root)
        self.cfg = config or SimConfig()
        self.rng = np.random.default_rng(seed)
        self.car = DiffDriveCar(self.car_params, slip=self.cfg.slip, rng=self.rng)
        self.renderer = FastRenderer(self.track, self.cam_params, rng=self.rng)
        self.lane = self.track.lanes[self.cfg.lane]
        self._reset_state()

    # ---------------- ciclo ----------------

    def _reset_state(self) -> None:
        self.t = 0.0
        self.seq = 0
        self._queue: deque[tuple[float, float, float]] = deque()
        self._applied = (0.0, 0.0)
        self._odo = [0.0, 0.0, 0.0]
        self._last_ticks = (0, 0)
        self._tl_green_at = 0.0
        self._tl_yellow_at = 0.0

    def reset(self, seed: int | None = None, pose: tuple[float, float, float] | None = None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
            self.car.rng = self.rng
            self.renderer.rng = self.rng
        self._reset_state()
        st = self.track.start
        x, y, yaw = pose or (st["x"], st["y"], st["yaw"])
        self.car.reset(x, y, yaw)
        self._odo = [x, y, yaw]
        self._tl_yellow_at = float(self.rng.uniform(*self.cfg.tl_red_s))
        self._tl_green_at = self._tl_yellow_at + float(self.rng.uniform(*self.cfg.tl_yellow_s))
        if self.cfg.randomize_fx:
            self.renderer.fx = PostFx(
                brightness=float(self.rng.uniform(0.7, 1.3)),
                noise_std=float(self.rng.uniform(0.0, 6.0)),
                vignette=float(self.rng.uniform(0.0, 0.4)),
            )
        obs, objs = self._observe()
        return obs, {"truth": self._truth(objs), "telemetry": self._telemetry()}

    def step(self, v: float, omega: float):
        """Envia (v, ω) — o mesmo contrato do ESP32 — e avança um quadro."""
        self._queue.append((self.t + self.cfg.latency_s, float(v), float(omega)))
        frame_dt = 1.0 / self.cfg.camera_hz
        n = max(1, int(round(frame_dt / self.cfg.physics_dt)))
        dt = frame_dt / n
        for _ in range(n):
            while self._queue and self._queue[0][0] <= self.t:
                _, v_c, w_c = self._queue.popleft()
                self._applied = (v_c, w_c)
            self.car.command(*self._applied)
            self.car.step(dt)
            self._integrate_odometry()
            self.t += dt

        self.seq += 1
        obs, objs = self._observe()
        truth = self._truth(objs)
        done, reason = self._check_done(truth)
        return obs, self._telemetry(), truth, done, {"reason": reason}

    # ---------------- semáforo ----------------

    def traffic_light_state(self) -> str:
        if self.t < self._tl_yellow_at:
            return "vermelho"
        if self.t < self._tl_green_at:
            return "amarelo"
        return "verde"

    @property
    def green_at(self) -> float:
        return self._tl_green_at

    # ---------------- internos ----------------

    def _observe(self):
        s = self.car.state
        img, info = self.renderer.render((s.x, s.y, s.yaw), self.traffic_light_state())
        return img, info["objects"]

    def _integrate_odometry(self) -> None:
        """Odometria a partir dos ticks — com a resolução do encoder real."""
        s, p = self.car.state, self.car_params
        dl_t, dr_t = s.ticks_left - self._last_ticks[0], s.ticks_right - self._last_ticks[1]
        self._last_ticks = (s.ticks_left, s.ticks_right)
        m_per_tick = 2.0 * math.pi * p.wheel_radius / p.counts_per_rev
        dl, dr = dl_t * m_per_tick, dr_t * m_per_tick
        ds = 0.5 * (dl + dr)
        dpsi = (dr - dl) / p.track_width
        x, y, psi = self._odo
        x += ds * math.cos(psi + dpsi / 2)
        y += ds * math.sin(psi + dpsi / 2)
        self._odo = [x, y, wrap_angle(psi + dpsi)]

    def _telemetry(self) -> Telemetry:
        s = self.car.state
        return Telemetry(self.seq, self.t, *self._odo, s.v_left, s.v_right, s.ticks_left, s.ticks_right)

    def _truth(self, objs) -> Truth:
        s = self.car.state
        proj = self.lane.line.project(s.x, s.y)
        e_psi = wrap_angle(s.yaw - proj.heading)
        foot = self.car.footprint()
        on = bool(np.all(self.track.on_carpet(foot)))
        return Truth(
            t=round(self.t, 4), x=s.x, y=s.y, yaw=s.yaw, v=s.v,
            omega=(s.v_right - s.v_left) / self.car_params.track_width,
            lane=self.lane.id, s=proj.s, e_y=proj.e_y, e_psi=e_psi, curvature=proj.curvature,
            on_carpet=on, in_own_lane=abs(proj.e_y) <= self.lane.width / 2,
            traffic_light=self.traffic_light_state(), objects=objs,
        )

    def _check_done(self, tr: Truth) -> tuple[bool, str]:
        if not tr.on_carpet:
            return True, "saiu_da_pista"          # regulamento: penalidade de 10 s
        if self.t >= self.cfg.max_episode_s:
            return True, "tempo_maximo"
        return False, ""
