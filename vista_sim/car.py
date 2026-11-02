"""Cinemática diferencial com dinâmica de 1ª ordem por roda.

O que entra no modelo é só o que muda como o carro se move na pista
(CLAUDE.md §9, regra 9):

* cada roda segue a velocidade pedida com constante de tempo ``tau``
  (motor + PID do ESP32) e aceleração limitada;
* saturação na velocidade máxima do motor (~1,65 m/s a 11 V);
* deslizamento lateral opcional;
* encoder quantizado — a odometria que o Vista recebe tem a mesma
  resolução que a do carro real.

O rodízio central é apoio passivo e não entra na dinâmica.

Interface: o carro recebe ``(v, ω)``, exatamente como o ESP32 real:
``v_l = v − ω·b/2``, ``v_r = v + ω·b/2``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .config import CarParams
from .geometry import wrap_angle


@dataclass
class CarState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v_left: float = 0.0           # velocidade real da superfície da roda (m/s)
    v_right: float = 0.0
    ticks_left: int = 0
    ticks_right: int = 0
    _tick_frac_l: float = field(default=0.0, repr=False)
    _tick_frac_r: float = field(default=0.0, repr=False)

    @property
    def v(self) -> float:
        return 0.5 * (self.v_left + self.v_right)


class DiffDriveCar:
    def __init__(self, params: CarParams, slip: float = 0.0, rng: np.random.Generator | None = None) -> None:
        self.p = params
        self.slip = slip
        self.rng = rng or np.random.default_rng()
        self.state = CarState()
        self._cmd = (0.0, 0.0)

    def reset(self, x: float, y: float, yaw: float) -> CarState:
        self.state = CarState(x=x, y=y, yaw=yaw)
        self._cmd = (0.0, 0.0)
        return self.state

    def command(self, v: float, omega: float) -> None:
        """Mesmo contrato do ESP32: velocidade linear e angular."""
        self._cmd = (float(v), float(omega))

    def wheel_targets(self, v: float, omega: float) -> tuple[float, float]:
        b = self.p.track_width
        vl, vr = v - omega * b / 2.0, v + omega * b / 2.0
        vmax = self.p.wheel_speed_max
        # Satura preservando a curvatura (escala as duas rodas juntas):
        # é o que um PID com saturação faz na prática.
        peak = max(abs(vl), abs(vr))
        if peak > vmax:
            vl, vr = vl * vmax / peak, vr * vmax / peak
        return vl, vr

    def step(self, dt: float) -> CarState:
        s = self.state
        tl, tr = self.wheel_targets(*self._cmd)
        a_max = self.p.max_wheel_accel * dt
        alpha = 1.0 - math.exp(-dt / self.p.time_constant)
        for side, target in (("v_left", tl), ("v_right", tr)):
            cur = getattr(s, side)
            dv = float(np.clip((target - cur) * alpha, -a_max, a_max))
            setattr(s, side, cur + dv)

        v = s.v
        omega = (s.v_right - s.v_left) / self.p.track_width
        if self.slip > 0:
            omega *= 1.0 - self.slip * float(self.rng.random())

        # Integração exata para arco de círculo (estável em curva fechada)
        if abs(omega) < 1e-9:
            s.x += v * math.cos(s.yaw) * dt
            s.y += v * math.sin(s.yaw) * dt
        else:
            r = v / omega
            s.x += r * (math.sin(s.yaw + omega * dt) - math.sin(s.yaw))
            s.y -= r * (math.cos(s.yaw + omega * dt) - math.cos(s.yaw))
        s.yaw = wrap_angle(s.yaw + omega * dt)

        # Encoder quantizado
        per_m = self.p.counts_per_rev / (2.0 * math.pi * self.p.wheel_radius)
        s._tick_frac_l += s.v_left * dt * per_m
        s._tick_frac_r += s.v_right * dt * per_m
        dl, dr = int(math.floor(s._tick_frac_l)), int(math.floor(s._tick_frac_r))
        s.ticks_left += dl
        s.ticks_right += dr
        s._tick_frac_l -= dl
        s._tick_frac_r -= dr
        return s

    def footprint(self) -> np.ndarray:
        """Retângulo do carro no mundo (4×2), para checar saída de pista."""
        s, p = self.state, self.p
        hw = p.width / 2
        local = np.array([[p.front_overhang, hw], [p.front_overhang, -hw],
                          [-p.rear_overhang, -hw], [-p.rear_overhang, hw]])
        c, sn = math.cos(s.yaw), math.sin(s.yaw)
        rot = np.array([[c, -sn], [sn, c]])
        return local @ rot.T + np.array([s.x, s.y])
