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

from ..geometry import rounded_rect

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

    # ---------------- faixas (verdade de campo) ----------------
    # Mão DIREITA. No anel, sentido anti-horário → faixa EXTERNA.
    lanes: list[dict[str, Any]] = [
        {"id": "anel_antihorario", "closed": True, "width": ring / 2,
         "points": _pl(rounded_rect(0, 0, hx - ring / 4, hy - ring / 4, max(rc - ring / 4, 0.1), 0.02))},
        {"id": "anel_horario", "closed": True, "width": ring / 2,
         "points": _pl(rounded_rect(0, 0, hx - 3 * ring / 4, hy - 3 * ring / 4, max(rc - 3 * ring / 4, 0.1), 0.02)[::-1])},
    ]

    # ---------------- placas ----------------
    # Slots: um por aproximação de cruzamento do anel (sentido anti-horário).
    # Regra 3.5.2: 400 mm ANTES do cruzamento, à DIREITA, tangenciando a
    # calçada. No anel externo a direita é a borda — a Figura 9 mostra a
    # placa do lado de fora, como aqui.
    d, sb = reg["sign_diameter"], reg["sign_bottom_height"]
    off = reg["sign_before_crossing"]
    out = d / 2 + 0.03
    slots = [
        # (nome, x, y, rumo do carro que se aproxima)
        ("anel_topo_oeste", qx0 + off, hy + out, math.pi),
        ("anel_esq_sul", -hx - out, qy0 + off, -math.pi / 2),
        ("anel_base_leste", -qx0 - off, -hy - out, 0.0),
        ("anel_dir_norte", hx + out, -qy0 - off, math.pi / 2),
    ]
    # Exemplo de classes — Figura 9 é "meramente ilustrativa"; no dia da
    # prova a organização define. O simulador sorteia por cenário.
    example = ["virar_esquerda", "virar_direita", "seguir_em_frente", "proibido_seguir_em_frente"]
    signs = []
    for (name, x, y, car_heading), cls in zip(slots, example, strict=True):
        signs.append({
            "slot": name, "class": cls,
            "x": round(x, 4), "y": round(y, 4),
            "facing_yaw": round(car_heading + math.pi, 6),   # a face aponta para o carro
            "diameter": d, "bottom_height": sb,
        })

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
        "lanes": lanes,
        "signs": signs,
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
