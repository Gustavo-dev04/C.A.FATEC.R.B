"""Localização no mapa oficial: em que rua o carro está, e onde nela.

O carro conhece o mapa (``maps/robocar_master.yaml``, gerado das figuras do
regulamento — o mesmo arquivo do simulador) e a pose de largada. Daí:

* **topológico** — a rua atual (aresta = rua + sentido) só muda num
  cruzamento, e quem decide para onde vai é o próprio Vista; então a próxima
  rua é ``opcoes[manobra]``, sem procurar. O mesmo grafo dá as saídas
  possíveis de cada cruzamento, que ``rules.decide`` precisa;
* **métrico** — pose (x, y, ψ) no referencial do mapa, PREVISTA pela odometria
  (deslocamento dos encoders, rumo do giroscópio) e, na faixa, CORRIGIDA pela
  rede: ``e_y`` e ``e_ψ`` amarram a pose de lado e de rumo à faixa do mapa, e a
  distância ao cruzamento amarra ao longo dela. Dentro do cruzamento não há
  correção: só odometria, por pouco mais de 1 m.

**Escala do giroscópio, calibrada nas curvas.** A calibração parada (no
vermelho) tira o viés, mas não o fator de escala — a MPU6050 garante só ±3 %,
e 3 % de 90° são 2,7°: numa conversão isso vira ~5 cm de erro na saída. Cada
curva é uma medida de graça: o giro que o giroscópio contou contra o giro da
pose (amarrada pela rede à faixa do mapa antes e depois). Mínimos quadrados
com um prior fraco em 1,0; da segunda curva em diante o erro cai para < 1 %.

A pose é a do eixo traseiro (``base_link``); a faixa é seguida pelo CENTRO do
carro, ``center_x`` à frente (ADR 003) — como na verdade do simulador.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from vista_sim.config import CarParams
from vista_sim.controle import Percepcao
from vista_sim.geometry import Polyline, Projection, wrap_angle
from vista_sim.roads import Edge, RoadNetwork


@dataclass
class LocalizacaoConfig:
    ganho_lateral: float = 0.3          # por quadro, quanto do e_y da rede entra
    ganho_rumo: float = 0.3             # idem e_ψ
    ganho_longitudinal: float = 0.15    # idem distância ao cruzamento
    dist_confiavel_m: float = 2.5       # a rede satura em 3 m: só corrige abaixo disto
    curva_min_rad: float = 1.0          # só curvas maiores que isto calibram a escala do giroscópio
    escala_prior: float = 0.25          # peso do prior (1,0), em "curvas de 90°"
    escala_lim: float = 0.08            # escala estimada fica em 1 ± isto


class LocalizadorGrafo:
    def __init__(self, net: RoadNetwork, car: CarParams, cfg: LocalizacaoConfig | None = None) -> None:
        self.net = net
        self.d = car.center_x
        self.cfg = cfg or LocalizacaoConfig()
        self.x = self.y = self.psi = 0.0
        self.edge: Edge | None = None
        self.reiniciar_escala()

    def reiniciar_escala(self) -> None:
        w = self.cfg.escala_prior * (math.pi / 2) ** 2
        self._sxy, self._syy = w, w
        self.escala_giro = 1.0
        self.curvas_calibradas = 0

    # ---------------- estado ----------------

    def reset(self, x: float, y: float, psi: float, edge: Edge | None = None) -> None:
        """Largada: pose conhecida (o carro é posto na linha de largada)."""
        self.x, self.y, self.psi = x, y, psi
        self._odo: tuple[float, float, float] | None = None
        self._giro_bruto = 0.0          # Σ giro da odometria (desenrolado)
        self._psi_acum = 0.0            # Σ variação do rumo estimado (desenrolado)
        self._marca: tuple[float, float] | None = None
        cx, cy = self.centro
        self.entrar_na_rua(edge or self.net.edge_for_pose(cx, cy, psi)[0])

    def entrar_na_rua(self, edge: Edge) -> None:
        """Passou a andar na faixa de ``edge`` (largada ou saída de um cruzamento)."""
        self.edge = edge
        self._lane = Polyline(self.net.lane(edge), closed=False)
        cx, cy = self.centro
        pr = self._lane.project(cx, cy)
        self._s, self._curv = pr.s, pr.curvature

    @property
    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.psi

    @property
    def centro(self) -> tuple[float, float]:
        return self.x + self.d * math.cos(self.psi), self.y + self.d * math.sin(self.psi)

    @property
    def no(self) -> str:
        """Cruzamento à frente."""
        assert self.edge is not None
        return self.net.end_node(self.edge)

    def opcoes(self) -> dict[str, Edge]:
        """Manobras possíveis no cruzamento à frente → rua de saída."""
        assert self.edge is not None
        return self.net.options(self.edge)

    @property
    def comprimento_faixa(self) -> float:
        return self._lane.length

    @property
    def s(self) -> float:
        """Onde o centro do carro está ao longo da faixa atual (m)."""
        return self._s

    @property
    def dist_cruzamento(self) -> float:
        """Distância do centro do carro até a entrada do cruzamento à frente (m)."""
        return self._lane.length - self._s

    # ---------------- previsão (odometria) ----------------

    def prever(self, odo: tuple[float, float, float]) -> None:
        """Aplica o deslocamento da odometria desde o último quadro.

        ``odo`` é a pose da odometria da telemetria (encoders + giroscópio).
        Só a DIFERENÇA entre quadros é usada, no referencial do carro: assim
        as correções feitas aqui não brigam com a odometria.
        """
        if self._odo is None:
            self._odo = odo
            return
        x0, y0, p0 = self._odo
        dx, dy = odo[0] - x0, odo[1] - y0
        c, s = math.cos(p0), math.sin(p0)
        fwd, lat = c * dx + s * dy, -s * dx + c * dy
        bruto = wrap_angle(odo[2] - p0)
        self._odo = odo
        self._giro_bruto += bruto
        dpsi = bruto / self.escala_giro
        c, s = math.cos(self.psi), math.sin(self.psi)
        self.x += c * fwd - s * lat
        self.y += s * fwd + c * lat
        self.psi = wrap_angle(self.psi + dpsi)
        self._psi_acum += dpsi
        self.projetar()

    # ---------------- escala do giroscópio ----------------

    def marcar_curva(self) -> None:
        """Início de uma conversão (rumo já amarrado à faixa de chegada)."""
        self._marca = (self._giro_bruto, self._psi_acum)

    def fechar_curva(self) -> float | None:
        """Fim (rumo já amarrado à faixa de saída): atualiza a escala. Devolve a amostra."""
        if self._marca is None:
            return None
        d_bruto = self._giro_bruto - self._marca[0]
        d_est = self._psi_acum - self._marca[1]
        self._marca = None
        if abs(d_est) < self.cfg.curva_min_rad:
            return None
        self._sxy += d_bruto * d_est
        self._syy += d_est * d_est
        lim = self.cfg.escala_lim
        self.escala_giro = min(max(self._sxy / self._syy, 1.0 - lim), 1.0 + lim)
        self.curvas_calibradas += 1
        return d_bruto / d_est

    @property
    def curvatura(self) -> float:
        """Curvatura da faixa do mapa onde o centro do carro está (1/m)."""
        return self._curv

    def projetar(self) -> Projection:
        cx, cy = self.centro
        pr = self._lane.project(cx, cy, self._s - 0.3, self._s + 0.6)
        self._s, self._curv = pr.s, pr.curvature
        return pr

    # ---------------- correção (rede de faixa) ----------------

    def corrigir(self, p: Percepcao) -> None:
        """Amarra a pose à faixa do mapa com o que a rede (já filtrada) mede."""
        c = self.cfg
        pr = self.projetar()
        h = pr.heading
        corr = c.ganho_rumo * wrap_angle(h + p.e_psi - self.psi)
        self.psi = wrap_angle(self.psi + corr)
        self._psi_acum += corr
        lat = c.ganho_lateral * (p.e_y - pr.e_y)
        lon = 0.0
        if not p.no_cruzamento and p.dist_cruzamento < c.dist_confiavel_m:
            lon = c.ganho_longitudinal * ((self._lane.length - p.dist_cruzamento) - pr.s)
        cx, cy = self.centro
        cx += -math.sin(h) * lat + math.cos(h) * lon
        cy += math.cos(h) * lat + math.sin(h) * lon
        self.x = cx - self.d * math.cos(self.psi)
        self.y = cy - self.d * math.sin(self.psi)
        self.projetar()
