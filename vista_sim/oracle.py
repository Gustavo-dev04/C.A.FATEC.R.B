"""Piloto-oráculo: dirige com percepção PERFEITA, decide pelas placas e
comanda ROTAÇÃO DE RODA.

Não é o Vista — o Vista só vê a câmera e a telemetria. O oráculo existe para:

1. **gerar dados de treino**: ele dirige, o simulador grava imagem e rótulos.
   Com ``noise`` > 0 ele erra de propósito e corrige — os dados de
   RECUPERAÇÃO sem os quais um modelo de imitação sai da pista na primeira
   imprecisão;
2. **referência**: o desempenho do oráculo é o teto do Vista com o mesmo
   controlador.

O oráculo NÃO recebe a rota pronta. Ele conhece a malha viária e as placas (a
percepção perfeita), e em cada cruzamento decide com ``rules.decide`` — a mesma
função que a máquina de estados do Vista vai importar. O ambiente confere se
a decisão bate com a rota do cenário; se não bater, é ``desrespeitou_placa``.

Controle: o CENTRO do carro (14 cm à frente do eixo) persegue um alvo na
trilha, por linearização do ponto fora do eixo; depois (v, ω) vira rotação de
cada roda, ``ω_E,D = (v ∓ ω·b/2) / r`` — o que o ESP32 recebe. Controlar o
centro, e não o eixo, é o que faz o carro de 40 cm caber nas conversões sem
a frente raspar a borda nem a traseira cortar a quina.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .car import wheel_rpm
from .config import CarParams
from .geometry import wrap_angle
from .roads import Edge, RoadNetwork
from .route import RoutePath
from .rules import decide


@dataclass
class OracleConfig:
    k_lat: float = 4.0              # 1/s — ganho do erro lateral do centro do carro
    # Ganho × velocidade ≤ isto: com 0,1 s de atraso de comando e o ponto
    # controlado a 14 cm do eixo, k = 4 oscila e diverge a 0,8 m/s (40/40
    # rotas fora); k·v ≤ 2,4 não muda nada até 0,6 m/s e dá k = 3 a 0,8 (20/20).
    kv_max: float = 2.4
    ff_time: float = 0.12           # s — antecipa a curva (atraso de comando ~0,1 s)
    v_cruise: float = 0.50
    v_turn: float = 0.28
    turn_k: float = 1.2             # curvatura (1/m) à frente que pede v_turn
    brake_dist: float = 0.6         # m antes da curva em que já desacelera
    accel: float = 1.0              # m/s² (rampa do comando)
    # Ruído: desvio lateral alvo, sorteado e mantido por um tempo, para o
    # carro sair do centro e o oráculo corrigir (dados de recuperação).
    noise: float = 0.0              # m (desvio-padrão do alvo lateral)
    noise_hold_s: float = 1.5
    # Limites do desvio: à DIREITA fica o meio-fio — na via de 80 cm o centro
    # da faixa está a 20 cm dele e o carro tem 12 de meia largura (sobram 8).
    # À ESQUERDA fica a outra faixa: dá para ir mais (e ensina a voltar da contramão).
    noise_max_right: float = 0.035  # m
    noise_max_left: float = 0.06    # m
    noise_rate: float = 0.05        # m/s — o alvo desliza, não pula (pulo = guinada = quina na borda)


@dataclass
class Decision:
    node: str
    arrival: Edge
    sign: str | None
    options: list[str]
    maneuver: str
    t: float = 0.0
    meta: dict[str, Any] = field(default_factory=dict)


class OraclePilot:
    def __init__(self, net: RoadNetwork, signs: list[dict[str, Any]], car: CarParams,
                 cfg: OracleConfig | None = None, rng: np.random.Generator | None = None) -> None:
        self.net = net
        self.car = car
        self.cfg = cfg or OracleConfig()
        self.rng = rng or np.random.default_rng()
        # percepção perfeita: a placa de cada aproximação (todas, inclusive as
        # que não são da rota — o oráculo não sabe qual é a rota)
        self.sign_at: dict[Edge, str] = {tuple(s["arrival"]): s["class"] for s in signs}  # type: ignore[misc]
        self.decisions: list[Decision] = []
        self._offset = 0.0
        self._offset_cur = 0.0
        self._offset_until = 0.0
        self._v = 0.0

    # ---------------- plano local ----------------

    def reset(self, x: float, y: float, yaw: float) -> None:
        e, s = self.net.edge_for_pose(x, y, yaw)
        self.decisions = []
        self._edges = [e]
        self._start_s = s
        self._v = 0.0
        self._plan(0.0)

    def _decide(self, arrival: Edge, t: float) -> Edge:
        opts = self.net.options(arrival)
        sign = self.sign_at.get(arrival)
        m = decide(sign, set(opts))
        self.decisions.append(Decision(self.net.end_node(arrival), arrival, sign, sorted(opts), m, t))
        return opts[m]

    def _plan(self, t: float) -> None:
        """Caminho local: aresta atual + as duas próximas decididas pela placa."""
        while len(self._edges) < 3:
            self._edges.append(self._decide(self._edges[-1], t))
        self.path = RoutePath(self.net, self._edges, start_s=self._start_s)
        self._s = 0.0

    def _advance(self, x: float, y: float, t: float) -> None:
        """Passou do primeiro cruzamento do plano: descarta a aresta velha."""
        j = self.path.junctions[0]
        if self._s > j.s_out + 0.05:
            self._edges.pop(0)
            self._start_s = 0.0
            self._plan(t)
            pr = self.path.line.project(x, y, 0.0, 0.6)
            self._s = pr.s

    # ---------------- controle ----------------

    def act(self, x: float, y: float, yaw: float, t: float, go: bool = True, dt: float = 1 / 30) -> tuple[float, float]:
        """Devolve (rpm_esquerda, rpm_direita)."""
        c, p = self.cfg, self.car
        if not go:
            self._v = 0.0
            return 0.0, 0.0
        if c.noise > 0 and t >= self._offset_until:
            self._offset = float(np.clip(self.rng.normal(0.0, c.noise),
                                         -min(2 * c.noise, c.noise_max_right), min(2 * c.noise, c.noise_max_left)))
            self._offset_until = t + float(self.rng.uniform(0.5, 1.5)) * c.noise_hold_s

        # o ponto controlado é o CENTRO do carro, não o eixo
        d = p.center_x
        cx, cy = x + d * math.cos(yaw), y + d * math.sin(yaw)
        pr = self.path.project(cx, cy, self._s, back=0.3, ahead=0.8)
        self._s = pr.s
        self._advance(cx, cy, t)
        pr = self.path.project(cx, cy, self._s, back=0.3, ahead=0.8)
        line = self.path.line

        # velocidade: desacelera antes da conversão
        k_ahead = 0.0
        for ahead in np.arange(0.0, c.brake_dist + 1e-9, 0.1):
            _, _, h0 = line.point_at(pr.s + ahead)
            _, _, h1 = line.point_at(pr.s + ahead + 0.2)
            k_ahead = max(k_ahead, abs(wrap_angle(h1 - h0)) / 0.2)
        v_target = c.v_turn if k_ahead > c.turn_k else c.v_cruise
        step = c.accel * dt
        self._v = float(np.clip(v_target, self._v - step, self._v + step))
        v = self._v

        # Velocidade desejada do centro: tangente à trilha (adiantada do atraso
        # de comando) + correção proporcional do erro lateral. Mirar num alvo
        # à frente cortaria a curva por ~L²/2R — 7 cm numa conversão de 25 cm.
        fx, fy, th = line.point_at(pr.s)
        _, _, th_ff = line.point_at(pr.s + v * c.ff_time)
        # Nas conversões o desvio some (errar ali vira outra rua). O desvio
        # aplicado desliza até o alvo a ``noise_rate`` — nunca pula.
        alvo = self._offset * max(0.0, 1.0 - k_ahead / c.turn_k) if c.noise > 0 else 0.0
        rate = c.noise_rate * dt
        self._offset_cur += float(np.clip(alvo - self._offset_cur, -rate, rate))
        off = self._offset_cur
        fx -= off * math.sin(th)
        fy += off * math.cos(th)
        k = min(c.k_lat, c.kv_max / max(v, 1e-3))
        ux = v * math.cos(th_ff) + k * (fx - cx)
        uy = v * math.sin(th_ff) + k * (fy - cy)
        # Ponto fora do eixo (linearização por realimentação):
        #   Ṗ = v·(cos ψ, sen ψ) + ω·d·(−sen ψ, cos ψ)  ⇒  resolve (v, ω) para Ṗ = u
        cs, sn = math.cos(yaw), math.sin(yaw)
        v_ax = ux * cs + uy * sn
        omega = float(np.clip((-ux * sn + uy * cs) / d, -p.omega_max, p.omega_max))
        return wheel_rpm(max(v_ax, 0.0), omega, p.track_width, p.wheel_radius)
