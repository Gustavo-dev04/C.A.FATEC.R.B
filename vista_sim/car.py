"""Carro de tração diferencial — modelado inteiramente em ROTAÇÃO DE RODA.

O carro não tem direção. Ele anda reto quando as duas rodas giram igual e
curva quando giram diferente. O simulador reproduz a cadeia inteira, roda
por roda, como no carro real:

    rpm alvo ──► PID do ESP32 ──► duty (PWM) ──► motor ──► rotação da roda
        ▲                                                      │
        └──────────── rpm medido ◄── encoder (ticks) ◄─────────┘

    rotação das rodas ──► (raio, patinação) ──► velocidade no chão ──► movimento

Detalhes que existem no carro real e por isso estão aqui:

* **zona morta do motor** — abaixo de ``deadband_duty`` a roda não gira;
* **encoder quantizado** — o PID e a odometria só veem ticks inteiros;
* **raios diferentes** entre as rodas (tolerância) — o carro puxa para um lado
  mesmo com rpm igual, e o controle precisa compensar;
* **patinação** — a roda gira mais do que o carro anda; os ticks contam a
  rotação, então a odometria diverge da realidade, como no carro.

Interfaces:

* :meth:`DiffDriveCar.command_wheels` — **principal**: rpm de cada roda;
* :meth:`DiffDriveCar.command` — atalho ``(v, ω)`` que só converte para rpm,
  exatamente como o ESP32 faria (``ω_roda = (v ∓ ω·b/2) / r``).
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from .config import CarParams
from .geometry import wrap_angle

RPM = 60.0 / (2.0 * math.pi)        # rad/s → rpm


def wheel_rpm(v: float, omega: float, track_width: float, wheel_radius: float) -> tuple[float, float]:
    """(v, ω) → rpm (esquerda, direita): ``ω_roda = (v ∓ ω·b/2) / r``.

    É a única ponte entre "quero andar a v e girar a ω" e o que o carro de
    fato entende — rotação de cada roda.
    """
    b, r = track_width, wheel_radius
    return (v - omega * b / 2.0) / r * RPM, (v + omega * b / 2.0) / r * RPM


@dataclass
class Wheel:
    radius: float                    # raio real desta roda (com tolerância)
    slip: float = 0.0                # fração de patinação (0 = nenhuma)
    omega: float = 0.0               # rotação real (rad/s)
    target: float = 0.0              # setpoint do PID (rad/s)
    omega_meas: float = 0.0          # o que o encoder mede (rad/s)
    duty: float = 0.0                # saída do PID, [-1, 1]
    integ: float = 0.0
    prev_err: float = 0.0
    ticks: int = 0
    _frac: float = 0.0
    _hist: deque = field(default_factory=lambda: deque(maxlen=64))

    @property
    def rpm(self) -> float:
        return self.omega * RPM

    @property
    def rpm_meas(self) -> float:
        """rpm que o ESP32 mede pelo encoder (quantizado)."""
        return self.omega_meas * RPM

    @property
    def ground_speed(self) -> float:
        return self.omega * self.radius * (1.0 - self.slip)


@dataclass
class CarState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    left: Wheel = field(default_factory=lambda: Wheel(0.0325))
    right: Wheel = field(default_factory=lambda: Wheel(0.0325))

    @property
    def v(self) -> float:
        return 0.5 * (self.left.ground_speed + self.right.ground_speed)

    # Compatibilidade com quem lê velocidade de superfície por roda
    @property
    def v_left(self) -> float:
        return self.left.ground_speed

    @property
    def v_right(self) -> float:
        return self.right.ground_speed

    @property
    def ticks_left(self) -> int:
        return self.left.ticks

    @property
    def ticks_right(self) -> int:
        return self.right.ticks


class DiffDriveCar:
    def __init__(self, params: CarParams, rng: np.random.Generator | None = None,
                 randomize: bool = False) -> None:
        self.p = params
        self.rng = rng or np.random.default_rng()
        self.randomize = randomize
        self.state = CarState()
        self.t = 0.0
        self._pid_acc = 0.0

    # ---------------- ciclo ----------------

    def reset(self, x: float, y: float, yaw: float) -> CarState:
        r = self.p.wheel_radius
        dl = dr = 0.0
        sl = sr = 0.0
        if self.randomize:
            m = self.p.radius_mismatch
            dl, dr = self.rng.uniform(-m, m, 2)
            # patinação: mesma nas duas rodas (o piso é o mesmo); a assimetria
            # entre rodas já vem da tolerância de raio
            sl = sr = float(self.rng.uniform(0.0, self.p.wheel_slip))
        self.state = CarState(x=x, y=y, yaw=yaw,
                              left=Wheel(r * (1 + dl), slip=sl),
                              right=Wheel(r * (1 + dr), slip=sr))
        self.t = 0.0
        self._pid_acc = 0.0
        return self.state

    def command_wheels(self, rpm_left: float, rpm_right: float) -> None:
        """Setpoint de rotação de cada roda (rpm). É o que o PID persegue."""
        lim = self.p.target_limit * self.p.omega_wheel_max
        tl, tr = rpm_left / RPM, rpm_right / RPM
        peak = max(abs(tl), abs(tr))
        if peak > lim:                       # escala junto: preserva a curva
            tl, tr = tl * lim / peak, tr * lim / peak
        self.state.left.target, self.state.right.target = tl, tr

    def wheel_rpm_for(self, v: float, omega: float) -> tuple[float, float]:
        """Cinemática inversa, como o ESP32: (v, ω) → rpm de cada roda.

        Usa o raio NOMINAL — o ESP32 não sabe que uma roda é 0,5 % maior.
        """
        return wheel_rpm(v, omega, self.p.track_width, self.p.wheel_radius)

    def command(self, v: float, omega: float) -> None:
        """Atalho (v, ω): converte para rpm de cada roda e comanda."""
        self.command_wheels(*self.wheel_rpm_for(v, omega))

    def step(self, dt: float) -> CarState:
        s, p = self.state, self.p
        pid_dt = 1.0 / p.pid_hz
        self._pid_acc += dt
        while self._pid_acc >= pid_dt:        # o PID roda na taxa do ESP32
            self._pid_acc -= pid_dt
            for w in (s.left, s.right):
                self._measure(w)
                self._pid(w, pid_dt)

        alpha = 1.0 - math.exp(-dt / p.motor_tau)
        for w in (s.left, s.right):
            w.omega += (self._omega_steady(w.duty) - w.omega) * alpha
            # encoder conta a ROTAÇÃO (não o chão): patinação engana a odometria
            w._frac += w.omega * dt / (2.0 * math.pi) * p.counts_per_rev
            n = int(math.floor(w._frac))
            w.ticks += n
            w._frac -= n

        v = s.v
        yaw_rate = (s.right.ground_speed - s.left.ground_speed) / p.track_width
        if abs(yaw_rate) < 1e-9:
            s.x += v * math.cos(s.yaw) * dt
            s.y += v * math.sin(s.yaw) * dt
        else:
            rad = v / yaw_rate
            s.x += rad * (math.sin(s.yaw + yaw_rate * dt) - math.sin(s.yaw))
            s.y -= rad * (math.cos(s.yaw + yaw_rate * dt) - math.cos(s.yaw))
        s.yaw = wrap_angle(s.yaw + yaw_rate * dt)
        self.t += dt
        return s

    # ---------------- motor, encoder, PID ----------------

    def _omega_steady(self, duty: float) -> float:
        """Rotação de regime para um duty: zona morta + linear até o rpm livre."""
        db = self.p.deadband_duty
        mag = max(0.0, abs(duty) - db) / (1.0 - db)
        return math.copysign(mag, duty) * self.p.omega_wheel_max

    def _measure(self, w: Wheel) -> None:
        """rpm medido = Δticks numa janela — com a quantização do encoder."""
        w._hist.append((self.t, w.ticks))
        t0, k0 = w._hist[0]
        while len(w._hist) > 2 and self.t - w._hist[1][0] >= self.p.speed_window_s:
            w._hist.popleft()
            t0, k0 = w._hist[0]
        span = self.t - t0
        if span > 1e-9:
            w.omega_meas = (w.ticks - k0) / self.p.counts_per_rev * 2.0 * math.pi / span

    def _pid(self, w: Wheel, dt: float) -> None:
        p = self.p
        err = w.target - w.omega_meas
        ff = 0.0
        if p.feedforward and abs(w.target) > 1e-6:
            # duty que, em regime, daria o rpm pedido (inverte a zona morta)
            ff = math.copysign(p.deadband_duty + (1 - p.deadband_duty) * abs(w.target) / p.omega_wheel_max,
                               w.target)
        deriv = (err - w.prev_err) / dt
        w.prev_err = err
        raw = ff + p.kp * err + p.ki * w.integ + p.kd * deriv
        duty = max(-1.0, min(1.0, raw))
        # anti-windup: só integra se não saturou na mesma direção do erro
        if duty == raw or (duty > 0) != (err > 0):
            w.integ += err * dt
        if abs(w.target) < 1e-6 and abs(w.omega_meas) < 1e-6:
            w.integ = 0.0                      # parado: zera o integrador
            duty = 0.0
        w.duty = duty

    # ---------------- geometria ----------------

    def footprint(self, per_edge: int = 1) -> np.ndarray:
        """Contorno do carro no mundo. ``per_edge`` > 1 subdivide cada lado —
        para pegar uma quina de quarteirão entrando pela lateral do carro."""
        s, p = self.state, self.p
        hw = p.width / 2
        c4 = np.array([[p.front_overhang, hw], [p.front_overhang, -hw],
                       [-p.rear_overhang, -hw], [-p.rear_overhang, hw]])
        if per_edge > 1:
            k = (np.arange(per_edge) / per_edge)[:, None]
            local = np.concatenate([c4[i] + (c4[(i + 1) % 4] - c4[i]) * k for i in range(4)])
        else:
            local = c4
        c, sn = math.cos(s.yaw), math.sin(s.yaw)
        rot = np.array([[c, -sn], [sn, c]])
        return local @ rot.T + np.array([s.x, s.y])
