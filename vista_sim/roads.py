"""Malha viária: ruas, cruzamentos, faixas, conversões e posição das placas.

Modelo:

* **nó** = cruzamento (T no anel externo, ou cruz no centro da cidade);
* **rua** = linha central de um nó a outro, com largura. Toda rua é de mão
  dupla com 2 faixas (inferido da rota das Figuras 10/11);
* **aresta** = uma rua percorrida num sentido: ``(rua, para_frente)``.

A faixa de uma aresta é a metade DIREITA da rua (mão direita, regulamento
3.4.1), recortada onde começa a área do cruzamento. Atravessar um cruzamento é
uma curva de Bézier da faixa que chega à faixa que sai.

A placa de cada aproximação fica **400 mm antes do cruzamento, à direita,
tangenciando a calçada** (regulamento 3.5.2) — calculada aqui, não escrita à
mão, para valer em qualquer mapa.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import numpy as np

from .geometry import Polyline, resample, wrap_angle

Edge = tuple[str, bool]                 # (id da rua, sentido a→b)

TURN_DEG = 35.0                         # |Δrumo| abaixo disso = "frente"
UTURN_DEG = 145.0                       # acima disso = retorno (não permitido)


@dataclass
class Node:
    id: str
    x: float
    y: float


@dataclass
class Road:
    id: str
    a: str
    b: str
    points: np.ndarray                  # linha central de a até b
    width: float

    def __post_init__(self) -> None:
        self.points = resample(np.asarray(self.points, dtype=np.float64), 0.02, closed=False)
        d = np.diff(self.points, axis=0)
        self._s = np.concatenate([[0.0], np.cumsum(np.hypot(d[:, 0], d[:, 1]))])
        self.length = float(self._s[-1])

    def heading_at_end(self, end: str) -> float:
        """Rumo da rua SAINDO do nó ``end`` (apontando para dentro da rua).

        Base de 40 cm: uma dobrinha logo na saída do nó (estreitamento ao lado
        da ilha) não pode transformar "em frente" em "direita".
        """
        p = self.points
        k = 20
        d = p[min(k, len(p) - 1)] - p[0] if end == "a" else p[max(-k - 1, -len(p))] - p[-1]
        return math.atan2(d[1], d[0])

    def slice(self, s0: float, s1: float) -> np.ndarray:
        """Trecho da linha central entre os comprimentos de arco s0 e s1."""
        ts = np.linspace(s0, s1, max(2, int((s1 - s0) / 0.02) + 1))
        return np.stack([np.interp(ts, self._s, self.points[:, 0]),
                         np.interp(ts, self._s, self.points[:, 1])], axis=1)

    def pose_at(self, s: float) -> tuple[float, float, float]:
        s = min(max(s, 0.0), self.length)
        x = float(np.interp(s, self._s, self.points[:, 0]))
        y = float(np.interp(s, self._s, self.points[:, 1]))
        h0 = np.interp(max(s - 0.02, 0), self._s, self.points[:, 0]), np.interp(max(s - 0.02, 0), self._s, self.points[:, 1])
        h1 = np.interp(min(s + 0.02, self.length), self._s, self.points[:, 0]), np.interp(min(s + 0.02, self.length), self._s, self.points[:, 1])
        return x, y, math.atan2(h1[1] - h0[1], h1[0] - h0[0])


def offset_right(points: np.ndarray, d: float) -> np.ndarray:
    """Desloca a polilinha ``d`` metros para a DIREITA do sentido de percurso."""
    p = np.asarray(points, dtype=np.float64)
    t = np.gradient(p, axis=0)
    n = np.hypot(t[:, 0], t[:, 1])[:, None]
    t = t / np.maximum(n, 1e-12)
    right = np.stack([t[:, 1], -t[:, 0]], axis=1)
    return p + right * d


def bezier(p0, h0, p3, h3, n: int = 40) -> np.ndarray:
    """Cúbica de p0 (rumo h0) a p3 (rumo h3).

    Numa conversão, cada controle é proporcional à sua "perna" — a distância
    até onde as duas tangentes se cruzam. Pernas iguais dão o arco exato
    (controle = (4/3)·tg(θ/4)·R); pernas diferentes (rua de 1,0 m entrando
    numa de 0,8 m) não fazem a curva passar do canto e voltar.
    """
    p0, p3 = np.asarray(p0, float), np.asarray(p3, float)
    t0 = np.array([math.cos(h0), math.sin(h0)])
    t3 = np.array([math.cos(h3), math.sin(h3)])
    chord = float(np.hypot(*(p3 - p0)))
    turn = abs(wrap_angle(h3 - h0))
    k0 = k3 = chord / 3.0
    if turn >= math.radians(TURN_DEG):
        # p0 + a·t0 = p3 − b·t3
        a, b = np.linalg.solve(np.column_stack([t0, t3]), p3 - p0)
        f = 4.0 / 3.0 * math.tan(turn / 4.0) / math.tan(turn / 2.0)
        if a > 0 and b > 0:
            k0, k3 = f * a, f * b
        else:
            radius = chord / (2.0 * math.sin(turn / 2.0))
            k0 = k3 = 4.0 / 3.0 * math.tan(turn / 4.0) * radius
    p1 = p0 + k0 * t0
    p2 = p3 - k3 * t3
    t = np.linspace(0.0, 1.0, n)[:, None]
    return (1 - t) ** 3 * p0 + 3 * (1 - t) ** 2 * t * p1 + 3 * (1 - t) * t**2 * p2 + t**3 * p3


class RoadNetwork:
    def __init__(self, data: dict[str, Any]) -> None:
        self.nodes = {n["id"]: Node(n["id"], float(n["x"]), float(n["y"])) for n in data["nodes"]}
        self.roads = {r["id"]: Road(r["id"], r["a"], r["b"], np.asarray(r["points"]), float(r["width"]))
                      for r in data["roads"]}
        self._trim = {(rid, end): self._compute_trim(rid, end)
                      for rid in self.roads for end in ("a", "b")}

    # ---------------- topologia ----------------

    def _roads_at(self, node: str) -> list[tuple[Road, str]]:
        out = []
        for r in self.roads.values():
            if r.a == node:
                out.append((r, "a"))
            if r.b == node:
                out.append((r, "b"))
        return out

    def _compute_trim(self, rid: str, end: str) -> float:
        """Recorte da rua no cruzamento = meia largura das ruas TRANSVERSAIS.

        Ruas colineares (a continuação do outro lado) não contam: elas não
        ocupam a área do cruzamento no sentido desta rua.
        """
        r = self.roads[rid]
        node = r.a if end == "a" else r.b
        h = r.heading_at_end(end)
        trim = 0.0
        for q, qend in self._roads_at(node):
            if q.id == rid and qend == end:
                continue
            if abs(math.sin(q.heading_at_end(qend) - h)) > 0.6:
                trim = max(trim, q.width / 2)
        return trim

    def is_junction(self, node: str) -> bool:
        return len(self._roads_at(node)) >= 3

    def arrivals(self, node: str) -> list[Edge]:
        return [(r.id, end == "b") for r, end in self._roads_at(node)]

    def departures(self, node: str) -> list[Edge]:
        return [(r.id, end == "a") for r, end in self._roads_at(node)]

    def end_node(self, e: Edge) -> str:
        r = self.roads[e[0]]
        return r.b if e[1] else r.a

    def start_node(self, e: Edge) -> str:
        r = self.roads[e[0]]
        return r.a if e[1] else r.b

    def heading_in(self, e: Edge) -> float:
        """Rumo de quem chega ao nó final da aresta."""
        r = self.roads[e[0]]
        return wrap_angle(r.heading_at_end("b" if e[1] else "a") + math.pi)

    def heading_out(self, e: Edge) -> float:
        r = self.roads[e[0]]
        return r.heading_at_end("a" if e[1] else "b")

    def maneuver(self, arrival: Edge, departure: Edge) -> str | None:
        if arrival[0] == departure[0]:
            return None                                     # retorno
        d = math.degrees(wrap_angle(self.heading_out(departure) - self.heading_in(arrival)))
        if abs(d) < TURN_DEG:
            return "frente"
        if abs(d) >= UTURN_DEG:
            return None
        return "esquerda" if d > 0 else "direita"

    def options(self, arrival: Edge) -> dict[str, Edge]:
        """Manobras possíveis ao chegar pelo ``arrival`` → aresta de saída."""
        node = self.end_node(arrival)
        out: dict[str, Edge] = {}
        for dep in self.departures(node):
            m = self.maneuver(arrival, dep)
            if m is not None:
                out[m] = dep
        return out

    # ---------------- geometria das faixas ----------------

    def lane(self, e: Edge) -> np.ndarray:
        """Faixa da DIREITA da aresta, recortada nos dois cruzamentos."""
        r = self.roads[e[0]]
        ta, tb = self._trim[(r.id, "a")], self._trim[(r.id, "b")]
        pts = r.slice(ta, r.length - tb)
        if not e[1]:
            pts = pts[::-1]
        return offset_right(pts, r.width / 4)

    def movement(self, arrival: Edge, departure: Edge) -> np.ndarray:
        """Trajeto dentro do cruzamento, da faixa que chega à faixa que sai.

        É o trajeto do CENTRO do carro, não do eixo (ver ``oracle.py``).
        """
        a, d = self.lane(arrival), self.lane(departure)
        h0 = math.atan2(*(a[-1] - a[-3])[::-1])
        h3 = math.atan2(*(d[2] - d[0])[::-1])
        return bezier(a[-1], h0, d[0], h3)

    def edge_for_pose(self, x: float, y: float, yaw: float) -> tuple[Edge, float]:
        """Aresta cuja faixa passa mais perto da pose, no mesmo sentido."""
        best, best_d, best_s = None, 1e9, 0.0
        for rid in self.roads:
            for fwd in (True, False):
                lane = Polyline(self.lane((rid, fwd)), closed=False)
                pr = lane.project(x, y)
                if abs(wrap_angle(pr.heading - yaw)) > math.radians(45):
                    continue
                # Distância REAL ao ponto projetado — não o erro lateral, que
                # dá zero para um ponto colinear com o prolongamento da faixa.
                fx, fy, _ = lane.point_at(pr.s)
                dist = math.hypot(x - fx, y - fy)
                if dist < best_d:
                    best, best_d, best_s = (rid, fwd), dist, pr.s
        if best is None:
            raise ValueError("nenhuma faixa compatível com a pose")
        return best, best_s

    # ---------------- placas e faixas de pedestre ----------------

    def approach_frame(self, arrival: Edge) -> tuple[np.ndarray, float, float]:
        """(ponto na linha central na entrada do cruzamento, rumo de chegada, largura)."""
        r = self.roads[arrival[0]]
        end = "b" if arrival[1] else "a"
        trim = self._trim[(r.id, end)]
        s = r.length - trim if arrival[1] else trim
        x, y, h = r.pose_at(s)
        heading = h if arrival[1] else wrap_angle(h + math.pi)
        return np.array([x, y]), heading, r.width

    def sign_pose(self, arrival: Edge, before: float = 0.400, outward: float = 0.08,
                  is_road: Callable[[float, float], bool] | None = None) -> dict[str, float]:
        """Regra 3.5.2: 400 mm ANTES do cruzamento, à DIREITA, na calçada.

        Os 400 mm são medidos AO LONGO da rua (que pode ter dobra, como as
        vias em volta da ilha), e "à direita" usa o rumo da rua naquele ponto.
        ``is_road`` (opcional) empurra a placa para fora até sair do asfalto —
        ao lado da ilha redonda a borda reta da via deixa uma fresta.
        """
        r = self.roads[arrival[0]]
        end = "b" if arrival[1] else "a"
        trim = self._trim[(r.id, end)]
        s = r.length - trim - before if arrival[1] else trim + before
        x, y, h = r.pose_at(s)
        h = h if arrival[1] else wrap_angle(h + math.pi)
        right = np.array([math.sin(h), -math.cos(h)])
        off = r.width / 2 + outward
        if is_road is not None:
            extra = 0.0
            while extra < 0.4 and is_road(*(np.array([x, y]) + right * (off + extra))):
                extra += 0.01
            off += extra
        px, py = np.array([x, y]) + right * off
        return {"x": float(px), "y": float(py), "facing_yaw": float(wrap_angle(h + math.pi))}

    def crosswalk(self, rid: str, end: str, gap: float, depth: float,
                  stripe: float, spacing: float) -> list[np.ndarray]:
        """Zebra atravessando a rua inteira, logo antes do cruzamento.

        As listras correm no sentido da rua (como numa zebra real) e são
        distribuídas pela largura.
        """
        r = self.roads[rid]
        trim = self._trim[(rid, end)]
        s_mid = trim + gap + depth / 2 if end == "a" else r.length - trim - gap - depth / 2
        x, y, h = r.pose_at(s_mid)
        along = np.array([math.cos(h), math.sin(h)])
        across = np.array([-math.sin(h), math.cos(h)])
        n = max(1, int((r.width - spacing) // (stripe + spacing)))
        total = n * stripe + (n - 1) * spacing
        polys = []
        for i in range(n):
            c = -total / 2 + stripe / 2 + i * (stripe + spacing)
            ctr = np.array([x, y]) + across * c
            hw, hd = stripe / 2, depth / 2
            polys.append(np.array([ctr + along * hd + across * hw, ctr + along * hd - across * hw,
                                   ctr - along * hd - across * hw, ctr - along * hd + across * hw]))
        return polys
