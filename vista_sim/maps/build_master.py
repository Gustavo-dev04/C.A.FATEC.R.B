"""Gera o mapa vetorial oficial da minicidade a partir do regulamento.

    python -m vista_sim.maps.build_master            # escreve maps/robocar_master.yaml

FONTE: regulamento RoboCar Race 2026 (docs/regulamento/), lido inclusive nas
figuras — a extração de texto do PDF não pega as cotas desenhadas.

Cada número aqui é de um de três tipos, e o mapa gerado registra qual:

* ``regulamento`` — está escrito ou cotado no regulamento;
* ``medido``      — medido nas figuras do regulamento ou nos .obj do simulador
                    do organizador (github.com/JulioZanotto/robocar_bot);
* ``inferido``    — dedução nossa, a confirmar com a organização.

Divergências encontradas com o simulador do organizador (que o próprio README
chama de "réplica ainda em desenvolvimento"):

* placa: o .obj tem Ø300 com centro a 550 mm; o regulamento diz Ø150 com a
  BASE a 400 mm (centro a 475). **Vale o regulamento.**
* linha central: o .obj é amarelo; a Figura 7 do regulamento mostra tracejado
  BRANCO. **Vale o regulamento.**
* classes de placa: o .obj tem 3 (direita/esquerda/frente); a Figura 8 do
  regulamento mostra 6 (as 3 + suas versões de proibição).
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from ..geometry import Polyline, resample, rounded_rect

# --------------------------------------------------------------------------
# Parâmetros
# --------------------------------------------------------------------------

REGULATION: dict[str, Any] = {
    # 3.4.1: "O tamanho total da cidade será de 9,0 m × 13,0 m"
    "size_x": 13.0,
    "size_y": 9.0,
    # Figura 6, cotas em cm:
    #   "100" entre a borda e os quarteirões         → anel externo
    #   "100 | 100" dos dois lados do canteiro VERTICAL → vias norte-sul
    #   "80" entre quarteirão e canteiro HORIZONTAL     → vias leste-oeste
    #   "80 | Fatec | 80" entre as pontas dos canteiros e a ilha central
    "ring_width": 1.00,
    "street_width_ns": 1.00,
    "street_width_ew": 0.80,
    "island_gap": 0.80,
    # Legenda da Figura 7: "quarteirões são de 2,0 × 4,0 metros"
    "block_x": 4.0,
    "block_y": 2.0,
    # 3.5.2 — placas
    "sign_diameter": 0.150,
    "sign_bottom_height": 0.400,     # "altura do chão até o ponto INFERIOR"
    "sign_before_crossing": 0.400,
    # 3.6 — semáforo (cotas da figura; 390 vai do chão ao CENTRO da amarela)
    "tl_box_width": 0.140,
    "tl_box_height": 0.470,
    "tl_lens_diameter": 0.110,
    "tl_lens_spacing": 0.140,
    "tl_yellow_center_height": 0.390,
    "tl_distance_from_start": 1.00,
}

INFERRED: dict[str, Any] = {
    # Figura 6 mostra cantos externos arredondados; o raio não é cotado.
    "outer_corner_radius": 1.00,
    # Espessura da faixa pintada não é cotada no regulamento.
    "line_width": 0.05,
    "dash_on": 0.30,
    "dash_off": 0.30,
    # Canteiros acompanham a extensão dos quarteirões (Figura 6: as barras
    # verdes começam e terminam alinhadas com os quarteirões). Com isso, a
    # cota "80" até a ilha fixa o raio dela em 0,70 m (Ø1,4). O .obj do
    # organizador tem Ø0,57 — DIVERGE; confirmar na pista.
    # Ruas internas de MÃO DUPLA: na Figura 10/11, a rota desce pelo lado
    # OESTE da via a leste do canteiro — só faz sentido mantendo a direita
    # numa via de mão dupla. CONFIRMAR com a organização.
    "inner_street_two_way": True,
    # Largada/chegada: no anel de cima, à direita (Figuras 6, 9, 10).
    # Medido na Figura 10: x ≈ +3,9 m. Rota sai para OESTE.
    "start_line_x": 3.90,
    # Semáforo do lado direito do carro, logo fora da borda externa.
    "tl_lateral_offset": 0.10,
    # O box de chegada não é descrito no regulamento.
    "box_length": 0.60,
}


def _rect(x0: float, x1: float, y0: float, y1: float) -> list[list[float]]:
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


def _circle(cx: float, cy: float, r: float, n: int = 48) -> list[list[float]]:
    return [[cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n)] for i in range(n)]


def _pl(points: np.ndarray) -> list[list[float]]:
    return [[round(float(x), 4), round(float(y), 4)] for x, y in points]


# --------------------------------------------------------------------------
# Construção
# --------------------------------------------------------------------------


def build(reg: dict[str, Any] = REGULATION, inf: dict[str, Any] = INFERRED) -> dict[str, Any]:
    hx, hy = reg["size_x"] / 2, reg["size_y"] / 2           # 6,5 e 4,5
    ring = reg["ring_width"]
    st_ns, st_ew = reg["street_width_ns"], reg["street_width_ew"]
    bx, by = reg["block_x"], reg["block_y"]
    lw = inf["line_width"]

    # Região interna ao anel
    ix, iy = hx - ring, hy - ring                             # 5,5 e 3,5
    # Quarteirões nos cantos, encostados no anel
    qx0, qy0 = ix - bx, iy - by                               # 1,5 e 1,5
    # O que sobra entre os quarteirões = via + canteiro + via
    mx = qx0 - st_ns                                          # 0,5 → canteiro vertical de 1,0
    my = qy0 - st_ew                                          # 0,7 → canteiro horizontal de 1,4
    if mx <= 0 or my <= 0:
        raise ValueError("cotas incompatíveis: as vias não cabem entre os quarteirões")
    island_r = min(qx0, qy0) - reg["island_gap"]              # 0,70
    if island_r <= 0:
        raise ValueError("cotas incompatíveis: não sobra espaço para a ilha central")

    obstacles: list[dict[str, Any]] = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            xa, xb = sorted((sx * qx0, sx * ix))
            ya, yb = sorted((sy * qy0, sy * iy))
            obstacles.append({"kind": "quadra", "polygon": _rect(xa, xb, ya, yb), "height": 0.05})
    # Canteiros centrais (verdes nas figuras)
    for sx in (-1, 1):
        xa, xb = sorted((sx * qx0, sx * ix))
        obstacles.append({"kind": "canteiro", "polygon": _rect(xa, xb, -my, my), "height": 0.03})
    for sy in (-1, 1):
        ya, yb = sorted((sy * qy0, sy * iy))
        obstacles.append({"kind": "canteiro", "polygon": _rect(-mx, mx, ya, yb), "height": 0.03})
    obstacles.append({"kind": "ilha", "polygon": _circle(0.0, 0.0, island_r), "height": 0.10})

    # ---------------- linhas pintadas ----------------
    lines: list[dict[str, Any]] = []
    rc = inf["outer_corner_radius"]
    # Borda externa contínua
    lines.append({
        "kind": "borda", "style": "solid", "width": lw, "closed": True,
        "points": _pl(rounded_rect(0, 0, hx - lw / 2, hy - lw / 2, rc - lw / 2)),
    })
    # Centro do anel: tracejado branco (Figura 7)
    lines.append({
        "kind": "centro", "style": "dashed", "width": lw, "closed": True,
        "dash": [inf["dash_on"], inf["dash_off"]],
        "points": _pl(rounded_rect(0, 0, hx - ring / 2, hy - ring / 2, max(rc - ring / 2, 0.05))),
    })
    # Meio-fio: contorno branco de cada quarteirão/canteiro, do lado da rua
    for ob in obstacles:
        if ob["kind"] == "ilha":
            pts = np.asarray(_circle(0, 0, island_r + lw / 2))
        else:
            p = np.asarray(ob["polygon"])
            x0, y0 = p.min(axis=0) - lw / 2
            x1, y1 = p.max(axis=0) + lw / 2
            pts = np.asarray(_rect(x0, x1, y0, y1))
        lines.append({"kind": "meio_fio", "style": "solid", "width": lw, "closed": True, "points": _pl(pts)})
    # Centro das ruas internas (se mão dupla)
    if inf["inner_street_two_way"]:
        cy_ = my + st_ew / 2                                   # 1,1
        cx_ = mx + st_ns / 2                                   # 1,0
        for sy in (-1, 1):
            for xa, xb in ((qx0, ix), (-ix, -qx0)):
                lines.append({"kind": "centro", "style": "dashed", "width": lw, "closed": False,
                              "dash": [inf["dash_on"], inf["dash_off"]],
                              "points": _pl(np.array([[xa, sy * cy_], [xb, sy * cy_]]))})
        for sx in (-1, 1):
            for ya, yb in ((qy0, iy), (-iy, -qy0)):
                lines.append({"kind": "centro", "style": "dashed", "width": lw, "closed": False,
                              "dash": [inf["dash_on"], inf["dash_off"]],
                              "points": _pl(np.array([[sx * cx_, ya], [sx * cx_, yb]]))})

    # ---------------- malha viária ----------------
    # Linha central de cada rua, de cruzamento a cruzamento. Toda rua é de mão
    # dupla com 2 faixas (inferido); a faixa de cada sentido é a metade direita.
    cy_ = my + st_ew / 2                                       # 1,1: vias leste-oeste
    cx_ = mx + st_ns / 2                                       # 1,0: vias norte-sul
    xc = (island_r + qx0) / 2                                  # 1,1: conector ao lado da ilha
    rx, ry = hx - ring / 2, hy - ring / 2                      # 6,0 e 4,0: centro do anel
    nodes = {
        # T-junções no anel externo
        "T_no": (-cx_, ry), "T_ne": (cx_, ry), "B_so": (-cx_, -ry), "B_se": (cx_, -ry),
        "L_no": (-rx, cy_), "L_so": (-rx, -cy_), "R_ne": (rx, cy_), "R_se": (rx, -cy_),
        # cruzamentos em volta da ilha central
        "C_no": (-cx_, cy_), "C_ne": (cx_, cy_), "C_so": (-cx_, -cy_), "C_se": (cx_, -cy_),
    }
    roads: list[dict[str, Any]] = []
    # Anel: corta a linha central fechada nos 8 cruzamentos, sentido anti-horário
    ring_line = Polyline(resample(rounded_rect(0, 0, rx, ry, max(rc - ring / 2, 0.05), 0.01), 0.01, True), True)
    ring_ids = ["R_ne", "T_ne", "T_no", "L_no", "L_so", "B_so", "B_se", "R_se"]
    s_of = {k: ring_line.project(*nodes[k]).s for k in ring_ids}
    ring_ids.sort(key=s_of.get)
    for i, a in enumerate(ring_ids):
        b = ring_ids[(i + 1) % len(ring_ids)]
        s0, s1 = s_of[a], s_of[b]
        if s1 <= s0:
            s1 += ring_line.length
        pts = [ring_line.point_at(s)[:2] for s in np.linspace(s0, s1, max(3, int((s1 - s0) / 0.02)))]
        pts[0], pts[-1] = nodes[a], nodes[b]
        roads.append({"id": f"anel_{a}_{b}", "a": a, "b": b, "width": ring, "points": _pl(np.asarray(pts))})

    def straight(rid: str, a: str, b: str, width: float, via: list | None = None) -> None:
        pts = [nodes[a], *(via or []), nodes[b]]
        roads.append({"id": rid, "a": a, "b": b, "width": width, "points": _pl(np.asarray(pts, float))})

    # vias norte-sul (1,0 m) entre o anel e o centro
    straight("ns_no", "T_no", "C_no", st_ns)
    straight("ns_ne", "T_ne", "C_ne", st_ns)
    straight("ns_so", "C_so", "B_so", st_ns)
    straight("ns_se", "C_se", "B_se", st_ns)
    # vias leste-oeste (0,8 m) entre o anel e o centro
    straight("lo_no", "L_no", "C_no", st_ew)
    straight("lo_ne", "C_ne", "R_ne", st_ew)
    straight("lo_so", "L_so", "C_so", st_ew)
    straight("lo_se", "C_se", "R_se", st_ew)
    # em volta da ilha central: as vias passam pela folga de 80 cm (Figura 6)
    gap = reg["island_gap"]
    straight("centro_n", "C_no", "C_ne", gap)
    straight("centro_s", "C_so", "C_se", gap)
    # A via N-S (1,0 m) estreita para 0,8 m ao lado da ilha: a linha central
    # desloca 10 cm. A dobra fica a 45° DENTRO do cruzamento — fora dele a
    # faixa já é reta, e a conversão não ganha um "gancho".
    dk = xc - cx_
    straight("centro_o", "C_no", "C_so", gap, [(-xc, cy_ - dk), (-xc, -cy_ + dk)])
    straight("centro_l", "C_ne", "C_se", gap, [(xc, cy_ - dk), (xc, -cy_ + dk)])

    network = {
        "nodes": [{"id": k, "x": round(v[0], 4), "y": round(v[1], 4)} for k, v in nodes.items()],
        "roads": roads,
    }

    # Placas: a POSIÇÃO de cada uma sai da regra 3.5.2 aplicada à malha
    # (vista_sim.roads.RoadNetwork.sign_pose). A CLASSE é definida por cenário
    # (vista_sim.scenario), porque a rota só é revelada no dia da prova.
    sign_spec = {
        "diameter": reg["sign_diameter"],
        "bottom_height": reg["sign_bottom_height"],
        "before_crossing": reg["sign_before_crossing"],
        "outward": 0.08,          # da beira da rua até o centro da placa — INFERIDO
    }

    # ---------------- largada, semáforo, box ----------------
    start_line = inf["start_line_x"]
    lane_y = hy - ring / 4                                     # 4,25: faixa externa
    front = 0.340                                              # frente do carro a partir de base_link (car.yaml)
    start = {"x": round(start_line + front, 4), "y": lane_y, "yaw": math.pi,
             "note": "base_link; a FRENTE do carro fica na linha de largada"}
    tl_x = start_line - reg["tl_distance_from_start"]
    traffic_light = {
        "x": round(tl_x, 4), "y": round(hy + inf["tl_lateral_offset"], 4),
        "facing_yaw": 0.0,
        "box_width": reg["tl_box_width"], "box_height": reg["tl_box_height"],
        "lens_diameter": reg["tl_lens_diameter"], "lens_spacing": reg["tl_lens_spacing"],
        "yellow_center_height": reg["tl_yellow_center_height"],
        "sequence": ["vermelho", "amarelo", "verde"],
    }
    box = {"polygon": _rect(start_line, start_line + inf["box_length"], hy - ring / 2, hy),
           "note": "INFERIDO — o regulamento não descreve o box"}

    return {
        "meta": {
            "name": "robocar_master",
            "source": "Regulamento RoboCar Race 2026 (versão 22/05/2026), incl. figuras",
            "generator": "vista_sim.maps.build_master",
            "regulation": REGULATION,
            "inferred": INFERRED,
            "frame": "origem no centro; x leste, y norte; metros",
        },
        "bounds": {"x_min": -hx, "x_max": hx, "y_min": -hy, "y_max": hy},
        "obstacles": obstacles,
        "lines": lines,
        "roads": network,
        "sign_spec": sign_spec,
        "traffic_light": traffic_light,
        "start": start,
        "box": box,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", default="maps/robocar_master.yaml")
    args = parser.parse_args(argv)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# GERADO por `python -m vista_sim.maps.build_master` — não editar à mão.\n"
        "# Mudou o regulamento ou uma inferência? Mude os parâmetros no gerador.\n"
    )
    out.write_text(header + yaml.safe_dump(build(), sort_keys=False, allow_unicode=True, width=120))
    print(f"mapa escrito em {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
