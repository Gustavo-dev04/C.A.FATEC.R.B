"""Gerador de cenários: rota, placas, faixas de pedestre, prédios, público e luz.

Tudo que a organização só revela no dia — e tudo que muda de um ginásio para
outro — é sorteado aqui, a partir de uma semente. Mesma semente, mesmo
cenário: é o que permite cenários de regressão (CLAUDE.md §5A).

Garantia central: **toda placa da rota leva, pela regra de decisão
(``vista_sim.rules.decide``), exatamente à manobra da rota.** O gerador confere
isso antes de devolver o cenário; um cenário inconsistente nunca sai daqui.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .geometry import point_in_polygon
from .roads import Edge, RoadNetwork
from .rules import OBLIGATION, PROHIBITION, decide, signs_that_force


@dataclass
class ScenarioConfig:
    # rota
    min_decisions: int = 6           # cruzamentos com decisão antes de voltar
    max_route_m: float = 45.0        # trecho "passeio" antes de procurar a volta
    max_total_m: float = 75.0        # rota inteira (o regulamento não dá o tamanho)
    turn_weight: float = 1.3         # >1 favorece entrar na cidade
    p_no_sign_straight: float = 0.35  # cruzamento da rota "em frente" sem placa
    p_prohibition: float = 0.35      # quando dá, usa placa de proibição
    p_distractor: float = 0.25       # placa em aproximação fora da rota
    # cenário físico
    p_crosswalk: float = 0.35        # por boca de rua num cruzamento
    p_buildings: float = 0.75        # por quarteirão
    max_buildings: int = 4
    people: tuple[int, int] = (0, 18)
    # luz
    lamps: tuple[int, int] = (2, 8)
    lamp_height: tuple[float, float] = (3.0, 9.0)
    ambient: tuple[float, float] = (0.25, 0.55)
    exposure: tuple[float, float] = (0.75, 1.25)


@dataclass
class Scenario:
    seed: int
    route: list[Edge]
    decisions: list[dict[str, Any]]
    signs: list[dict[str, Any]]
    crosswalks: list[np.ndarray]
    buildings: list[dict[str, Any]]
    people: list[dict[str, Any]]
    lighting: dict[str, Any]
    start_s: float = 0.0
    meta: dict[str, Any] = field(default_factory=dict)


# --------------------------------------------------------------------------
# rota
# --------------------------------------------------------------------------


def _lane_len(net: RoadNetwork, e: Edge) -> float:
    lane = net.lane(e)
    return float(np.sum(np.hypot(*np.diff(lane, axis=0).T)))


def _route_back(net: RoadNetwork, frm: Edge, goal: Edge, assigned: dict[Edge, str]) -> list[Edge] | None:
    """BFS de ``frm`` até entrar na aresta ``goal``, respeitando manobras já
    fixadas (uma aproximação não pode pedir duas manobras diferentes).

    Devolve as arestas DEPOIS de ``frm``, terminando em ``goal``.
    """
    if frm == goal:
        return []
    prev: dict[Edge, Edge] = {}
    seen = {frm}
    q = deque([frm])
    while q:
        e = q.popleft()
        opts = net.options(e)
        if e in assigned:
            opts = {assigned[e]: opts[assigned[e]]}
        for nxt in opts.values():
            if nxt in seen:
                continue
            seen.add(nxt)
            prev[nxt] = e
            if nxt == goal:
                path = [nxt]
                while path[-1] in prev and prev[path[-1]] != frm:
                    path.append(prev[path[-1]])
                return path[::-1]
            q.append(nxt)
    return None


def generate_route(net: RoadNetwork, start: Edge, start_s: float, rng: np.random.Generator,
                   cfg: ScenarioConfig) -> list[Edge]:
    """Passeio aleatório pela cidade e volta à largada. Uma aproximação usada
    duas vezes pede sempre a mesma manobra — senão a placa seria impossível."""
    for _ in range(300):
        assigned: dict[Edge, str] = {}
        route = [start]
        length = _lane_len(net, start) - start_s
        e = start
        while len(assigned) < cfg.min_decisions and length < cfg.max_route_m:
            opts = net.options(e)
            if e in assigned:
                m = assigned[e]
            else:
                names = sorted(opts)
                w = np.array([1.0 if n == "frente" else cfg.turn_weight for n in names])
                m = names[int(rng.choice(len(names), p=w / w.sum()))]
                assigned[e] = m
            e = opts[m]
            route.append(e)
            length += _lane_len(net, e)
        back = _route_back(net, e, start, assigned)
        if back is None:
            continue
        full = route + back
        # confere consistência ao longo da rota inteira
        seen: dict[Edge, Edge] = {}
        ok = True
        for a, b in zip(full[:-1], full[1:], strict=True):
            if a in seen and seen[a] != b:
                ok = False
                break
            seen[a] = b
        total = sum(_lane_len(net, x) for x in full[1:-1]) + _lane_len(net, start)
        if ok and total <= cfg.max_total_m:
            return full
    raise RuntimeError("não consegui gerar uma rota consistente")


# --------------------------------------------------------------------------
# placas
# --------------------------------------------------------------------------


def road_test(map_data: dict[str, Any]) -> Callable[[float, float], bool]:
    """(x, y) está no asfalto? (dentro da borda e fora de quarteirão, canteiro e ilha)"""
    border = np.asarray(next(ln for ln in map_data["lines"] if ln["kind"] == "borda")["points"], float)
    obstacles = [np.asarray(o["polygon"], float) for o in map_data["obstacles"]]

    def is_road(x: float, y: float) -> bool:
        X, Y = np.array([x]), np.array([y])
        if not point_in_polygon(X, Y, border)[0]:
            return False
        return not any(point_in_polygon(X, Y, ob)[0] for ob in obstacles)

    return is_road


def place_signs(net: RoadNetwork, route: list[Edge], spec: dict[str, Any], rng: np.random.Generator,
                cfg: ScenarioConfig, is_road: Callable[[float, float], bool] | None = None
                ) -> tuple[list[dict], list[dict]]:
    decisions, signs = [], []
    done: set[Edge] = set()
    for a, b in zip(route[:-1], route[1:], strict=True):
        opts = net.options(a)
        m = net.maneuver(a, b)
        dec = {"node": net.end_node(a), "arrival": list(a), "options": sorted(opts), "maneuver": m, "sign": None}
        if a not in done:
            done.add(a)
            sign = None
            if not (m == "frente" and rng.random() < cfg.p_no_sign_straight):
                cands = signs_that_force(m, set(opts))
                prohib = [c for c in cands if c in PROHIBITION]
                oblig = [c for c in cands if c in OBLIGATION]
                pool = prohib if (prohib and rng.random() < cfg.p_prohibition) else oblig
                sign = str(rng.choice(pool))
            if sign is not None:
                signs.append({**net.sign_pose(a, spec["before_crossing"], spec["outward"], is_road),
                              "class": sign, "diameter": spec["diameter"],
                              "bottom_height": spec["bottom_height"],
                              "node": dec["node"], "arrival": list(a), "on_route": True})
        dec["sign"] = next((s["class"] for s in signs if tuple(s["arrival"]) == a), None)
        # A garantia: a regra de decisão chega à manobra da rota.
        assert decide(dec["sign"], set(opts)) == m, (dec, opts)
        decisions.append(dec)

    # placas fora da rota: o Vista tem que ler só a da SUA aproximação
    route_arrivals = {tuple(d["arrival"]) for d in decisions}
    for nid in net.nodes:
        for arr in net.arrivals(nid):
            if arr in route_arrivals or rng.random() >= cfg.p_distractor:
                continue
            opts = set(net.options(arr))
            valid = [c for c in (*OBLIGATION, *PROHIBITION) if _decides(c, opts)]
            if valid:
                signs.append({**net.sign_pose(arr, spec["before_crossing"], spec["outward"], is_road),
                              "class": str(rng.choice(valid)), "diameter": spec["diameter"],
                              "bottom_height": spec["bottom_height"],
                              "node": nid, "arrival": list(arr), "on_route": False})
    return decisions, signs


def _decides(sign: str, opts: set[str]) -> bool:
    try:
        decide(sign, opts)
        return True
    except ValueError:
        return False


# --------------------------------------------------------------------------
# cenário físico
# --------------------------------------------------------------------------


def place_crosswalks(net: RoadNetwork, rng: np.random.Generator, cfg: ScenarioConfig) -> list[np.ndarray]:
    """Zebras nas bocas de rua. O regulamento NÃO fala de faixa de pedestre:
    posição e medidas são sorteadas (escala 1:10 de uma zebra real)."""
    polys: list[np.ndarray] = []
    for rid, road in net.roads.items():
        for end, node in (("a", road.a), ("b", road.b)):
            if not net.is_junction(node) or rng.random() >= cfg.p_crosswalk:
                continue
            polys += net.crosswalk(rid, end,
                                   gap=float(rng.uniform(0.02, 0.08)),
                                   depth=float(rng.uniform(0.22, 0.40)),
                                   stripe=float(rng.uniform(0.03, 0.05)),
                                   spacing=float(rng.uniform(0.03, 0.06)))
    return polys


def place_buildings(quadras: list[np.ndarray], rng: np.random.Generator, cfg: ScenarioConfig) -> list[dict]:
    """Maquetes nos quarteirões (regulamento 3.4.1). Recuadas da beira, para
    não ocupar a calçada onde ficam as placas."""
    out = []
    inset = 0.18
    palette = [(150, 150, 150), (120, 130, 150), (175, 165, 150), (110, 120, 130), (160, 150, 120), (140, 110, 100)]
    for q in quadras:
        if rng.random() >= cfg.p_buildings:
            continue
        x0, y0 = q.min(axis=0) + inset
        x1, y1 = q.max(axis=0) - inset
        placed: list[tuple[float, float, float, float]] = []
        for _ in range(int(rng.integers(1, cfg.max_buildings + 1)) * 6):
            if len(placed) >= cfg.max_buildings:
                break
            w, d = rng.uniform(0.25, 1.2), rng.uniform(0.25, 0.9)
            if w > x1 - x0 or d > y1 - y0:
                continue
            bx, by = rng.uniform(x0, x1 - w), rng.uniform(y0, y1 - d)
            box = (bx, by, bx + w, by + d)
            if any(not (box[2] + 0.05 < o[0] or o[2] + 0.05 < box[0] or box[3] + 0.05 < o[1] or o[3] + 0.05 < box[1])
                   for o in placed):
                continue
            placed.append(box)
            c = palette[int(rng.integers(len(palette)))]
            out.append({"footprint": np.array([[box[0], box[1]], [box[2], box[1]], [box[2], box[3]], [box[0], box[3]]]),
                        "height": float(rng.uniform(0.08, 0.60)),
                        "color": tuple(int(v * rng.uniform(0.85, 1.1)) for v in c)})
    return out


def place_people(bounds: dict[str, float], rng: np.random.Generator, cfg: ScenarioConfig) -> list[dict]:
    """Público e juízes em volta da pista. A 1:1 eles são gigantes perto da
    câmera a 156 mm, e as sombras deles atravessam a pista."""
    out = []
    hx, hy = bounds["x_max"], bounds["y_max"]
    for _ in range(int(rng.integers(cfg.people[0], cfg.people[1] + 1))):
        side = int(rng.integers(4))
        off = float(rng.uniform(0.35, 1.3))
        t = float(rng.uniform(-1, 1))
        x, y = [(hx + off, t * hy), (-hx - off, t * hy), (t * hx, hy + off), (t * hx, -hy - off)][side]
        out.append({"x": x, "y": y, "radius": float(rng.uniform(0.17, 0.26)),
                    "height": float(rng.uniform(1.50, 1.90)),
                    "shirt": tuple(int(v) for v in rng.integers(30, 230, 3)),
                    "pants": tuple(int(v) for v in rng.integers(20, 120, 3))})
    return out


def _kelvin_bgr(k: float) -> tuple[float, float, float]:
    """Tom aproximado de uma lâmpada branca (BGR, normalizado no verde)."""
    warm, neutral, cool = (0.70, 0.86, 1.00), (0.90, 0.95, 1.00), (1.00, 0.97, 0.95)
    if k <= 5000:
        t = (k - 3000) / 2000
        a, b = warm, neutral
    else:
        t = (k - 5000) / 1500
        a, b = neutral, cool
    t = min(max(t, 0.0), 1.0)
    return tuple(float(a[i] + (b[i] - a[i]) * t) for i in range(3))  # type: ignore[return-value]


def place_lights(rng: np.random.Generator, cfg: ScenarioConfig) -> dict[str, Any]:
    """Lâmpadas do ginásio. "Mesmo branca", a luz vem de pontos discretos e o
    ambiente bloqueia parte dela — daí as sombras e a luz desigual."""
    lamps = []
    for _ in range(int(rng.integers(cfg.lamps[0], cfg.lamps[1] + 1))):
        lamps.append({"x": float(rng.uniform(-9, 9)), "y": float(rng.uniform(-7, 7)),
                      "z": float(rng.uniform(*cfg.lamp_height)),
                      "intensity": float(rng.uniform(0.5, 1.5)),
                      "bgr": _kelvin_bgr(float(rng.uniform(3500, 6500))),
                      "size": float(rng.uniform(0.1, 0.8))})
    return {"lamps": lamps, "ambient": float(rng.uniform(*cfg.ambient)),
            "ambient_bgr": _kelvin_bgr(float(rng.uniform(4500, 6000))),
            "exposure": float(rng.uniform(*cfg.exposure))}


def generate(map_data: dict[str, Any], net: RoadNetwork, seed: int, cfg: ScenarioConfig | None = None) -> Scenario:
    cfg = cfg or ScenarioConfig()
    rng = np.random.default_rng(seed)
    st = map_data["start"]
    start_edge, start_s = net.edge_for_pose(st["x"], st["y"], st["yaw"])
    route = generate_route(net, start_edge, start_s, rng, cfg)
    decisions, signs = place_signs(net, route, map_data["sign_spec"], rng, cfg, road_test(map_data))
    quadras = [np.asarray(o["polygon"]) for o in map_data["obstacles"] if o["kind"] == "quadra"]
    return Scenario(
        seed=seed, route=route, decisions=decisions, signs=signs,
        crosswalks=place_crosswalks(net, rng, cfg),
        buildings=place_buildings(quadras, rng, cfg),
        people=place_people(map_data["bounds"], rng, cfg),
        lighting=place_lights(rng, cfg),
        start_s=start_s,
        meta={"entra_na_cidade": any(d["node"].startswith("C_") for d in decisions),
              "cruzamentos": len(decisions)},
    )
