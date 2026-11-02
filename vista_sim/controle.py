"""Controle a partir da PERCEPÇÃO: o que a rede vê → rpm de cada roda.

Lógica que vai para o carro, como ``rules.py``. Entra o que a rede de faixa
estima a partir da imagem — erro lateral ``e_y`` e de rumo ``e_ψ`` do centro
do carro, curvatura à frente ``κ``, distância ao cruzamento e se já está
dentro dele — e a manobra decidida pela placa. Sai o rpm de cada roda.

A lei é a mesma do oráculo (ADR 003), escrita no referencial do carro:

    φ   = −e_ψ                          rumo da trilha visto do carro
    u   = v·(cos, sen)(φ + κ·v·t_ff)    anda tangente à trilha (adiantada)
        + k·(−e_y)·(−sen φ, cos φ)      e corrige o erro lateral
    v_eixo = u_x,   ω = u_y / d         d = centro do carro à frente do eixo

Com a verdade de campo no lugar da rede, este piloto é o teto do que a rede
pode atingir (``tools/avaliar_modelo.py --fonte verdade``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .car import wheel_rpm
from .config import CarParams


@dataclass
class Percepcao:
    e_y: float                 # m, + = centro do carro à esquerda da trilha
    e_psi: float               # rad, + = apontando para a esquerda da trilha
    kappa: float               # 1/m, curvatura média nos próximos 40 cm
    dist_cruzamento: float     # m até a entrada do próximo cruzamento (0 = chegou)
    no_cruzamento: bool        # já dentro da área do cruzamento


@dataclass
class ControleConfig:
    k_lat: float = 4.0         # 1/s
    ff_time: float = 0.12      # s — atraso de comando ~0,1 s
    v_cruise: float = 0.50
    v_turn: float = 0.28
    turn_k: float = 1.2        # |κ| acima disso = curva
    brake_dist: float = 0.6    # m antes de um cruzamento com conversão
    accel: float = 1.0         # m/s²


class PilotoFaixa:
    def __init__(self, car: CarParams, cfg: ControleConfig | None = None) -> None:
        self.car = car
        self.cfg = cfg or ControleConfig()
        self._v = 0.0

    def reset(self) -> None:
        self._v = 0.0

    def act(self, p: Percepcao, manobra: str, go: bool, dt: float = 1 / 30) -> tuple[float, float]:
        """Devolve (rpm_esquerda, rpm_direita)."""
        c, car = self.cfg, self.car
        if not go:
            self._v = 0.0
            return 0.0, 0.0
        curva = abs(p.kappa) > c.turn_k or (
            manobra != "frente" and (p.no_cruzamento or p.dist_cruzamento < c.brake_dist))
        v_target = c.v_turn if curva else c.v_cruise
        step = c.accel * dt
        self._v = float(np.clip(v_target, self._v - step, self._v + step))
        v = self._v

        phi = -p.e_psi
        th = phi + p.kappa * v * c.ff_time
        ux = v * math.cos(th) + c.k_lat * p.e_y * math.sin(phi)
        uy = v * math.sin(th) - c.k_lat * p.e_y * math.cos(phi)
        omega = float(np.clip(uy / car.center_x, -car.omega_max, car.omega_max))
        return wheel_rpm(max(ux, 0.0), omega, car.track_width, car.wheel_radius)
