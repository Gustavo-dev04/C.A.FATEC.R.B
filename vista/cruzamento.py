"""Atravessar o cruzamento pela curva do MAPA, com odometria.

Dentro do cruzamento a rede de faixa não tem o que ver: a curva que o carro
deve fazer não está pintada no chão. Medido no diário (09/11): lá dentro a
rede estima κ ≈ 0 contra 1,3 1/m de verdade, e e_ψ ≈ 0° contra −30…−41°.

Mas a curva ESTÁ no mapa: ``net.movement(chegada, saída)`` é o trajeto do
centro do carro da faixa que chega à faixa que sai — o mesmo que o oráculo
segue. Então:

1. na entrada, com a pose já amarrada à faixa pela rede (``mapa.py``), monta
   o trajeto: fim da faixa de chegada → conversão → começo da faixa de saída;
2. o centro do carro segue esse trajeto com a pose da ODOMETRIA (encoders +
   giroscópio) — pouco mais de 1 m sem correção;
3. já dentro da faixa de saída, devolve o controle para a rede.

Lei de controle: a do oráculo (ADR 003) — anda tangente ao trajeto (adiantado
do atraso de comando) e corrige o erro lateral do centro do carro; o ponto
fora do eixo vira (v, ω) por linearização e daí rpm de cada roda.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from vista_sim.car import wheel_rpm
from vista_sim.config import CarParams
from vista_sim.roads import Edge, RoadNetwork
from vista_sim.route import RoutePath


@dataclass
class CruzamentoConfig:
    k_lat: float = 4.0          # 1/s, como o oráculo e o PilotoFaixa
    ff_time: float = 0.12       # s
    v_frente: float = 0.50      # m/s atravessando reto
    v_curva: float = 0.28       # m/s na conversão
    accel: float = 1.0          # m/s²
    antes_m: float = 0.15       # trajeto começa este tanto atrás do carro (faixa de chegada)
    depois_m: float = 0.80      # trecho da faixa de saída incluído no trajeto
    devolve_m: float = 0.30     # devolve à rede depois de entrar isto na faixa de saída
    max_aprox: float = 0.35     # correção lateral ≤ isto × v (como o PilotoFaixa)
    kv_max: float = 2.4         # ganho × velocidade ≤ isto (ver OracleConfig.kv_max)


class ManobraOdometria:
    def __init__(self, net: RoadNetwork, car: CarParams, cfg: CruzamentoConfig | None = None) -> None:
        self.net = net
        self.car = car
        self.cfg = cfg or CruzamentoConfig()
        self.ativa = False
        self.terminou = False
        self.v = 0.0

    def iniciar(self, chegada: Edge, saida: Edge, manobra: str, s_chegada: float, v_atual: float) -> None:
        """``s_chegada``: onde o centro do carro está na faixa de chegada (m)."""
        c = self.cfg
        self.manobra = manobra
        self.chegada, self.saida = chegada, saida
        self.trajeto = RoutePath(self.net, [chegada, saida], start_s=max(s_chegada - c.antes_m, 0.0),
                                 end_s=c.depois_m)
        j = self.trajeto.junctions[0]
        self.s_fim = j.s_out + c.devolve_m
        self._s = 0.0
        self.v = v_atual
        self.ativa, self.terminou = True, False

    def act(self, x: float, y: float, psi: float, dt: float = 1 / 30) -> tuple[float, float]:
        """Pose ESTIMADA (odometria) → (rpm_esquerda, rpm_direita)."""
        c, p = self.cfg, self.car
        d = p.center_x
        cx, cy = x + d * math.cos(psi), y + d * math.sin(psi)
        line = self.trajeto.line
        pr = line.project(cx, cy, self._s - 0.2, self._s + 0.6)
        self._s = pr.s
        if self._s >= self.s_fim:
            self.terminou = True

        v_alvo = c.v_frente if self.manobra == "frente" else c.v_curva
        step = c.accel * dt
        self.v = float(np.clip(v_alvo, self.v - step, self.v + step))
        v = self.v

        fx, fy, _ = line.point_at(pr.s)
        _, _, th_ff = line.point_at(pr.s + v * c.ff_time)
        k = min(c.k_lat, c.kv_max / max(v, 1e-3))
        lx, ly = k * (fx - cx), k * (fy - cy)
        n = math.hypot(lx, ly)
        lim = c.max_aprox * v
        if n > lim:
            lx, ly = lx * lim / n, ly * lim / n
        ux = v * math.cos(th_ff) + lx
        uy = v * math.sin(th_ff) + ly
        cs, sn = math.cos(psi), math.sin(psi)
        v_ax = ux * cs + uy * sn
        omega = float(np.clip((-ux * sn + uy * cs) / d, -p.omega_max, p.omega_max))
        return wheel_rpm(max(v_ax, 0.0), omega, p.track_width, p.wheel_radius)
