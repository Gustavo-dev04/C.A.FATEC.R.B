"""Rota como um caminho só: faixas e conversões emendadas, com os cruzamentos marcados.

A rota do cenário é uma lista de arestas (rua + sentido). Para medir o carro
(``e_y``, ``e_ψ``, progresso) ela vira UMA polilinha aberta:

    faixa(largada, a partir da largada) → conversão → faixa → conversão → …
    → faixa(largada, até a largada)

e cada conversão guarda o trecho ``[s_in, s_out]`` que ocupa. Como a rota pode
passar duas vezes pelo mesmo cruzamento, a projeção é sempre feita numa JANELA
em torno do progresso anterior (:meth:`RoutePath.project`).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .geometry import Polyline, Projection
from .roads import Edge, RoadNetwork


@dataclass
class Junction:
    index: int
    node: str
    arrival: Edge
    departure: Edge
    maneuver: str
    options: list[str]
    sign: str | None
    s_in: float                 # onde a faixa de chegada termina
    s_out: float                # onde a faixa de saída começa


def _length(pts: np.ndarray) -> float:
    return float(np.sum(np.hypot(*np.diff(pts, axis=0).T))) if len(pts) > 1 else 0.0


def _cut(pts: np.ndarray, s0: float | None, s1: float | None) -> np.ndarray:
    """Trecho da polilinha entre os comprimentos de arco s0 e s1."""
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
    a = 0.0 if s0 is None else max(0.0, s0)
    b = s[-1] if s1 is None else min(s[-1], s1)
    inner = (s > a) & (s < b)
    ts = np.concatenate([[a], s[inner], [b]])
    return np.stack([np.interp(ts, s, pts[:, 0]), np.interp(ts, s, pts[:, 1])], axis=1)


class RoutePath:
    def __init__(self, net: RoadNetwork, edges: list[Edge], start_s: float = 0.0,
                 end_s: float | None = None, signs: dict[Edge, str | None] | None = None) -> None:
        """``edges[0]`` começa em ``start_s``; ``edges[-1]`` termina em ``end_s``
        (``None`` = fim da faixa)."""
        self.net = net
        self.edges = list(edges)
        signs = signs or {}
        pieces: list[np.ndarray] = []
        self.junctions: list[Junction] = []
        self.lane_ranges: list[tuple[Edge, float, float]] = []
        widths: list[tuple[float, float]] = []          # (s até onde vale, meia faixa)
        s = 0.0

        def add(pts: np.ndarray) -> None:
            nonlocal s
            if pieces:
                last = pieces[-1][-1]
                if np.allclose(pts[0], last, atol=1e-6):
                    pts = pts[1:]
                if len(pts) == 0:
                    return
                s += float(np.hypot(*(pts[0] - last)))
            s += _length(pts)
            pieces.append(pts)

        n = len(self.edges)
        for i, e in enumerate(self.edges):
            lane = net.lane(e)
            s0 = start_s if i == 0 else None
            s1 = end_s if i == n - 1 else None
            seg = _cut(lane, s0, s1)
            a = s
            add(seg)
            self.lane_ranges.append((e, a, s))
            widths.append((s, net.roads[e[0]].width / 4))
            if i < n - 1:
                nxt = self.edges[i + 1]
                mv = net.movement(e, nxt)
                s_in = s
                add(mv)
                opts = net.options(e)
                self.junctions.append(Junction(
                    index=len(self.junctions), node=net.end_node(e), arrival=e, departure=nxt,
                    maneuver=net.maneuver(e, nxt) or "?", options=sorted(opts), sign=signs.get(e),
                    s_in=s_in, s_out=s))
                widths.append((s, min(net.roads[e[0]].width, net.roads[nxt[0]].width) / 4))
        self.points = np.vstack(pieces)
        self.line = Polyline(self.points, closed=False)
        self.length = self.line.length
        self._w_end = np.array([w[0] for w in widths])
        self._w_val = np.array([w[1] for w in widths])

    # ---------------- consultas ----------------

    def project(self, x: float, y: float, s_hint: float, back: float = 0.5, ahead: float = 1.0) -> Projection:
        return self.line.project(x, y, s_hint - back, s_hint + ahead)

    def half_lane(self, s: float) -> float:
        """Meia largura da faixa no ponto ``s`` (metade de meia rua)."""
        i = min(int(np.searchsorted(self._w_end, s)), len(self._w_val) - 1)
        return float(self._w_val[i])

    def next_junction(self, s: float) -> Junction | None:
        """Primeiro cruzamento cuja conversão ainda não terminou."""
        for j in self.junctions:
            if j.s_out > s:
                return j
        return None

    def junction_near(self, s: float, before: float = 0.15, after: float = 0.8) -> Junction | None:
        for j in self.junctions:
            if j.s_in - before <= s <= j.s_out + after:
                return j
        return None

    def edge_at(self, s: float) -> Edge | None:
        """Aresta da faixa em ``s``; ``None`` dentro de um cruzamento."""
        for e, a, b in self.lane_ranges:
            if a <= s <= b:
                return e
        return None
