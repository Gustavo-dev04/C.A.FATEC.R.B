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
    # A correção lateral (k·e_y) é limitada a esta fração da velocidade — no
    # máximo ~20° de ângulo de aproximação. Sem isso, 5 cm de erro de uma vez
    # (saindo de um cruzamento) viram 1,6 rad/s de giro: a frente do carro
    # varre para o lado e sai do tapete.
    max_aprox: float = 0.35
    kv_max: float = 2.4        # ganho × velocidade ≤ isto (ver OracleConfig.kv_max)


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
        lim = c.max_aprox * v                 # parado não corrige de lado (giraria no lugar)
        k = min(c.k_lat, c.kv_max / max(v, 1e-3))
        lat = float(np.clip(k * p.e_y, -lim, lim))
        ux = v * math.cos(th) + lat * math.sin(phi)
        uy = v * math.sin(th) - lat * math.cos(phi)
        omega = float(np.clip(uy / car.center_x, -car.omega_max, car.omega_max))
        return wheel_rpm(max(ux, 0.0), omega, car.track_width, car.wheel_radius)


# --------------------------------------------------------------------------
# Filtro: a rede corrige, o movimento do próprio carro prevê
# --------------------------------------------------------------------------


@dataclass
class FiltroConfig:
    ganho_faixa: float = 0.25        # quanto de cada medida da rede entra (na faixa)
    ganho_cruzamento: float = 0.12   # dentro do cruzamento a rede erra mais: confia menos
    reinicia_m: float = 0.10         # medida tão longe da previsão = algo mudou: aceita a medida


class FiltroFaixa:
    """Filtro complementar de ``e_y``/``e_ψ`` — lógica que vai para o carro.

    Entre um quadro e outro, o próprio carro diz como se moveu: ``v`` pelos
    encoders e ``ω`` pelo giroscópio (viés já descontado). Isso PREVÊ o novo
    erro; a rede só CORRIGE a previsão com um ganho pequeno. O ruído da rede
    (que o controle do centro do carro amplifica: 1 cm de ``e_y`` ≈ 0,3 rad/s)
    é atenuado sem atrasar a resposta a uma curva de verdade.

    Medido (verdade + ruído 0,8/2,5 cm, 20 rotas): 8/20 → 14/20 completas.

        ψ̇_rel = ω − κ·v
        ẏ     = v·sen(e_ψ) + ω·d·cos(e_ψ)      (d: centro do carro à frente do eixo)
    """

    def __init__(self, car: CarParams, cfg: FiltroConfig | None = None) -> None:
        self.d = car.center_x
        self.cfg = cfg or FiltroConfig()
        self.reset()

    def reset(self) -> None:
        self.e_y: float | None = None
        self.e_psi = 0.0

    def atualizar(self, medida: Percepcao, v: float, omega: float, dt: float,
                  kappa_atual: float | None = None) -> Percepcao:
        """``kappa_atual``: curvatura da faixa ONDE o carro está (do mapa, se
        houver). Sem ela, usa o κ da rede — que é a média dos próximos 40 cm e,
        chegando numa conversão, já "vê" a curva: a previsão giraria antes da
        hora e o rumo filtrado ficaria ~2° torto na entrada do cruzamento."""
        c = self.cfg
        k_prev = medida.kappa if kappa_atual is None else kappa_atual
        if self.e_y is None:
            self.e_y, self.e_psi = medida.e_y, medida.e_psi
        else:
            ep = self.e_psi + (omega - k_prev * v) * dt
            ey = self.e_y + (v * math.sin(self.e_psi) + omega * self.d * math.cos(self.e_psi)) * dt
            if abs(medida.e_y - ey) > c.reinicia_m:
                ey, ep = medida.e_y, medida.e_psi
            g = c.ganho_cruzamento if (medida.no_cruzamento or medida.dist_cruzamento < 0.3) else c.ganho_faixa
            self.e_y = ey + g * (medida.e_y - ey)
            self.e_psi = ep + g * (medida.e_psi - ep)
        return Percepcao(self.e_y, self.e_psi, medida.kappa, medida.dist_cruzamento, medida.no_cruzamento)
