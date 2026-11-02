"""Geometria 2D: polilinhas, retângulos arredondados e projeção em faixas.

Python puro + NumPy. É a base da verdade de campo: ``e_y`` e ``e_ψ`` que o
simulador exporta em todo frame saem de :meth:`Polyline.project`.

Convenção (igual ao carro): x para a frente/leste, y para a esquerda/norte,
ângulos em radianos, anti-horário positivo.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def wrap_angle(a: float) -> float:
    """Normaliza para (-π, π]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def rounded_rect(
    cx: float, cy: float, half_w: float, half_h: float, radius: float, arc_step: float = 0.05
) -> np.ndarray:
    """Retângulo de cantos arredondados, sentido ANTI-HORÁRIO.

    Começa na borda direita e percorre: direita↑, topo←, esquerda↓, base→.
    No anel externo isso é o sentido de quem larga indo para oeste — ver
    Figuras 10/11 do regulamento.

    Args:
        arc_step: comprimento máximo de cada segmento do arco, em metros.
    """
    r = max(0.0, min(radius, half_w, half_h))
    corners = [
        (cx + half_w - r, cy + half_h - r, 0.0),
        (cx - half_w + r, cy + half_h - r, 0.5 * math.pi),
        (cx - half_w + r, cy - half_h + r, math.pi),
        (cx + half_w - r, cy - half_h + r, 1.5 * math.pi),
    ]
    pts: list[tuple[float, float]] = []
    n = max(2, int(math.ceil(0.5 * math.pi * r / arc_step)) + 1) if r > 0 else 1
    for ox, oy, a0 in corners:
        if r == 0:
            pts.append((ox, oy))
            continue
        for i in range(n):
            a = a0 + 0.5 * math.pi * i / (n - 1)
            pts.append((ox + r * math.cos(a), oy + r * math.sin(a)))
    return np.asarray(pts, dtype=np.float64)


def resample(points: np.ndarray, step: float, closed: bool) -> np.ndarray:
    """Reamostra uma polilinha com espaçamento ~uniforme ``step``."""
    pts = np.asarray(points, dtype=np.float64)
    if closed:
        pts = np.vstack([pts, pts[:1]])
    seg = np.diff(pts, axis=0)
    seg_len = np.hypot(seg[:, 0], seg[:, 1])
    keep = seg_len > 1e-12
    pts = np.vstack([pts[:1], pts[1:][keep]])
    seg_len = seg_len[keep]
    s = np.concatenate([[0.0], np.cumsum(seg_len)])
    total = s[-1]
    n = max(2, int(round(total / step)) + (0 if closed else 1))
    targets = np.linspace(0.0, total, n, endpoint=not closed)
    x = np.interp(targets, s, pts[:, 0])
    y = np.interp(targets, s, pts[:, 1])
    return np.stack([x, y], axis=1)


@dataclass
class Projection:
    """Resultado da projeção de um ponto numa faixa."""

    s: float
    """Distância percorrida ao longo da faixa até o ponto projetado (m)."""

    e_y: float
    """Erro lateral (m). POSITIVO = carro à ESQUERDA do centro da faixa."""

    heading: float
    """Rumo da faixa no ponto projetado (rad)."""

    curvature: float
    """Curvatura com sinal (1/m). Positiva = curva à esquerda."""

    segment: int


class Polyline:
    """Polilinha orientada com consultas de projeção vetorizadas."""

    def __init__(self, points: np.ndarray, closed: bool) -> None:
        pts = np.asarray(points, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < 2:
            raise ValueError("polilinha precisa de pelo menos 2 pontos (N, 2)")
        self.points = pts
        self.closed = closed
        self._a = pts
        self._b = np.roll(pts, -1, axis=0) if closed else pts[1:]
        if not closed:
            self._a = pts[:-1]
        d = self._b - self._a
        self._len = np.hypot(d[:, 0], d[:, 1])
        self._len[self._len < 1e-12] = 1e-12
        self._dir = d / self._len[:, None]
        self._heading = np.arctan2(d[:, 1], d[:, 0])
        self._s0 = np.concatenate([[0.0], np.cumsum(self._len)[:-1]])
        self.length = float(self._len.sum())
        self._curv = self._segment_curvature()

    def _segment_curvature(self) -> np.ndarray:
        h = self._heading
        n = len(h)
        nxt = np.roll(h, -1) if self.closed else np.append(h[1:], h[-1])
        dh = (nxt - h + np.pi) % (2 * np.pi) - np.pi
        ds = 0.5 * (self._len + (np.roll(self._len, -1) if self.closed else np.append(self._len[1:], self._len[-1])))
        k = dh / np.maximum(ds, 1e-9)
        # suaviza: média com o vizinho anterior
        prv = np.roll(k, 1) if self.closed else np.insert(k[:-1], 0, k[0])
        return 0.5 * (k + prv) if n > 1 else k

    def project(self, x: float, y: float) -> Projection:
        """Projeta (x, y) no segmento mais próximo."""
        p = np.array([x, y])
        ap = p - self._a
        t = np.clip(np.einsum("ij,ij->i", ap, self._dir), 0.0, self._len)
        foot = self._a + self._dir * t[:, None]
        dist2 = np.sum((p - foot) ** 2, axis=1)
        i = int(np.argmin(dist2))
        d = self._dir[i]
        rel = p - foot[i]
        e_y = float(d[0] * rel[1] - d[1] * rel[0])  # cruzado: + = à esquerda
        return Projection(
            s=float(self._s0[i] + t[i]),
            e_y=e_y,
            heading=float(self._heading[i]),
            curvature=float(self._curv[i]),
            segment=i,
        )

    def point_at(self, s: float) -> tuple[float, float, float]:
        """Ponto e rumo a uma distância ``s`` ao longo da faixa."""
        s = s % self.length if self.closed else min(max(s, 0.0), self.length)
        i = int(np.searchsorted(self._s0, s, side="right") - 1)
        i = min(max(i, 0), len(self._s0) - 1)
        t = s - self._s0[i]
        x, y = self._a[i] + self._dir[i] * t
        return float(x), float(y), float(self._heading[i])


def point_in_polygon(x: np.ndarray, y: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Teste par-ímpar vetorizado. ``x``/``y`` de mesmo formato."""
    inside = np.zeros(np.shape(x), dtype=bool)
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        cond = (y0 > y) != (y1 > y)
        with np.errstate(divide="ignore", invalid="ignore"):
            xint = x0 + (y - y0) * (x1 - x0) / (y1 - y0)
        inside ^= cond & (x < xint)
    return inside
