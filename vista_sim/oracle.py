"""Piloto-oráculo: dirige usando a VERDADE de campo.

Não é o Vista — o Vista só vê a câmera e a telemetria. O oráculo existe para
duas coisas:

1. **gerar dados de treino**: ele dirige, o simulador grava a imagem e os
   rótulos. Com ``noise`` > 0 ele erra de propósito e corrige — o que produz
   os dados de RECUPERAÇÃO (carro fora do centro voltando) sem os quais
   qualquer modelo de imitação sai da pista na primeira imprecisão;
2. **referência**: o desempenho do oráculo é o teto do que o Vista pode
   atingir com o mesmo controlador.

Controle: pure pursuit para tração diferencial, ``ω = 2·v·sin(α) / L_d``
(CLAUDE.md §4) — a mesma lei que o Vista vai usar.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .geometry import Polyline, wrap_angle


@dataclass
class OracleConfig:
    lookahead: float = 0.35         # m — CLAUDE.md §4 sugere 0,3–0,4
    v_cruise: float = 0.50
    v_curve: float = 0.35
    curve_k: float = 1.2            # acima desta curvatura (1/m), reduz v
    omega_max: float = 3.0
    # Ruído: desvio lateral alvo, sorteado e mantido por um tempo, para o
    # carro sair do centro e o oráculo corrigir (dados de recuperação).
    noise: float = 0.0              # m (desvio-padrão do alvo lateral)
    noise_hold_s: float = 1.5


class OraclePilot:
    def __init__(self, lane: Polyline, cfg: OracleConfig | None = None,
                 rng: np.random.Generator | None = None) -> None:
        self.lane = lane
        self.cfg = cfg or OracleConfig()
        self.rng = rng or np.random.default_rng()
        self._offset = 0.0
        self._offset_until = 0.0

    def act(self, x: float, y: float, yaw: float, t: float, go: bool = True) -> tuple[float, float]:
        if not go:
            return 0.0, 0.0
        c = self.cfg
        if c.noise > 0 and t >= self._offset_until:
            self._offset = float(np.clip(self.rng.normal(0.0, c.noise), -2 * c.noise, 2 * c.noise))
            self._offset_until = t + float(self.rng.uniform(0.5, 1.5)) * c.noise_hold_s

        proj = self.lane.project(x, y)
        tx, ty, th = self.lane.point_at(proj.s + c.lookahead)
        # desloca o alvo lateralmente (ruído de recuperação)
        tx -= self._offset * math.sin(th)
        ty += self._offset * math.cos(th)

        dx, dy = tx - x, ty - y
        alpha = wrap_angle(math.atan2(dy, dx) - yaw)
        dist = max(math.hypot(dx, dy), 1e-3)

        # curvatura à frente define a velocidade
        _, _, th_far = self.lane.point_at(proj.s + 2.5 * c.lookahead)
        k_ahead = abs(wrap_angle(th_far - proj.heading)) / (2.5 * c.lookahead)
        v = c.v_curve if k_ahead > c.curve_k else c.v_cruise
        omega = 2.0 * v * math.sin(alpha) / dist
        return v, float(np.clip(omega, -c.omega_max, c.omega_max))
