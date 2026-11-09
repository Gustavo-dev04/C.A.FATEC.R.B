"""Iluminação: lâmpadas do ginásio, sombras do ambiente e do próprio carro.

"Mesmo branca", a luz de um ginásio vem de pontos discretos, e tudo que fica
entre a lâmpada e o chão faz sombra: as maquetes nos quarteirões (regulamento
3.4.1), o público e os juízes em volta da pista, o semáforo, as placas — e o
próprio carro. Para a câmera, uma sombra atravessando a faixa é um degrau de
brilho que um limiar fixo de "branco" confunde com borda de faixa.

Modelo (o mínimo que muda o que a câmera vê):

* cada lâmpada é um ponto a uma altura H; no chão, a irradiância cai como
  ``cos³θ`` (``= H³ / (H² + d²)^(3/2)``, normalizada para 1 logo abaixo);
* a sombra de um obstáculo (prisma vertical de z0 a z1) é a envoltória convexa
  das projeções da base e do topo a partir da lâmpada;
* penumbra: desfoque proporcional ao tamanho da lâmpada;
* luz ambiente constante (rebatimento nas paredes) e tom de cor por lâmpada;
* exposição travada (CLAUDE.md §F1): a câmera NÃO compensa — cena mais escura
  vira imagem mais escura.

O mapa de luz é calculado uma vez por cenário, em 2 cm/px, no mesmo
referencial da textura do chão — por quadro custa um ``remap`` pequeno.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

LIGHT_ONE = 128.0          # valor uint8 que representa luz = 1,0 (máx. ≈ 2,0)


@dataclass
class Occluder:
    """Prisma vertical de ``z0`` a ``z1`` sobre o polígono ``outline``."""

    outline: np.ndarray        # (N, 2) no mundo
    z0: float
    z1: float


def project_from_lamp(pts: np.ndarray, z: float, lamp: tuple[float, float, float]) -> np.ndarray:
    """Onde a sombra de pontos à altura ``z`` cai no chão, para a lâmpada dada."""
    lx, ly, lz = lamp
    if z >= lz - 1e-3:
        z = lz - 1e-3
    k = lz / (lz - z)
    return np.column_stack([lx + (pts[:, 0] - lx) * k, ly + (pts[:, 1] - ly) * k])


def shadow_polygon(oc: Occluder, lamp: tuple[float, float, float]) -> np.ndarray:
    """Envoltória convexa da sombra do prisma no chão."""
    pts = np.vstack([project_from_lamp(oc.outline, oc.z0, lamp),
                     project_from_lamp(oc.outline, oc.z1, lamp)]).astype(np.float32)
    return cv2.convexHull(pts).reshape(-1, 2)


def irradiance(x: np.ndarray, y: np.ndarray, lamp: dict[str, float]) -> np.ndarray:
    """Luz direta no chão: cos³ do ângulo com a vertical (1,0 logo abaixo)."""
    d2 = (x - lamp["x"]) ** 2 + (y - lamp["y"]) ** 2
    h = lamp["z"]
    return lamp["intensity"] * (h**3) / np.power(h * h + d2, 1.5)


class LightField:
    """Mapa de luz BGR do chão (uint8, 128 = 1,0) + sombra dinâmica do carro."""

    def __init__(self, lighting: dict[str, Any], occluders: list[Occluder],
                 x_min: float, x_max: float, y_min: float, y_max: float,
                 track_bounds: dict[str, float], res: float = 0.02) -> None:
        self.cfg = lighting
        self.x_min, self.y_max, self.res = x_min, y_max, res
        W = int(math.ceil((x_max - x_min) / res))
        H = int(math.ceil((y_max - y_min) / res))
        xs = x_min + (np.arange(W) + 0.5) * res
        ys = y_max - (np.arange(H) + 0.5) * res
        X, Y = np.meshgrid(xs, ys)

        amb = lighting["ambient"]
        light = np.empty((H, W, 3), np.float32)
        light[:] = np.array(lighting["ambient_bgr"], np.float32) * amb
        self._lamps = lighting["lamps"]
        for lamp in self._lamps:
            E = irradiance(X, Y, lamp).astype(np.float32)
            vis = np.ones((H, W), np.float32)
            lpos = (lamp["x"], lamp["y"], lamp["z"])
            for oc in occluders:
                poly = shadow_polygon(oc, lpos)
                px = np.round(self._to_px(poly) * 16).astype(np.int32)
                cv2.fillPoly(vis, [px], 0.0, cv2.LINE_AA, 4)
            sigma = max(0.6, lamp["size"] * 0.15 / res)
            vis = cv2.GaussianBlur(vis, (0, 0), sigma)
            light += (E * vis)[:, :, None] * np.array(lamp["bgr"], np.float32)

        # Normaliza pela mediana DENTRO da pista: "1,0" = luz típica da pista.
        inside = (track_bounds["x_min"] < X) & (track_bounds["x_max"] > X) & \
                 (track_bounds["y_min"] < Y) & (track_bounds["y_max"] > Y)
        ref = float(np.median(light[inside].mean(axis=1)))
        light *= lighting["exposure"] / max(ref, 1e-6)
        self.mean_level = float(light[inside].mean())
        self.map = np.clip(light * LIGHT_ONE, 0, 255).astype(np.uint8)
        self._ref = ref

    def _to_px(self, xy: np.ndarray) -> np.ndarray:
        return np.column_stack([(xy[:, 0] - self.x_min) / self.res, (self.y_max - xy[:, 1]) / self.res])

    def at(self, x: float, y: float) -> np.ndarray:
        """Luz BGR (1,0 = típica) num ponto do mundo."""
        H, W = self.map.shape[:2]
        u = int(min(max((x - self.x_min) / self.res, 0), W - 1))
        v = int(min(max((self.y_max - y) / self.res, 0), H - 1))
        return self.map[v, u].astype(np.float32) / LIGHT_ONE

    # ---------------- sombra do próprio carro ----------------

    def car_shadow(self, pose: tuple[float, float, float], body: np.ndarray, height: float,
                   grid_origin: tuple[float, float], grid_res: float, grid_shape: tuple[int, int],
                   max_lamps: int = 3) -> np.ndarray:
        """Escurecimento [0, 1] num grid NO REFERENCIAL DO CARRO.

        A sombra anda com o carro; por isso é desenhada no referencial dele e
        levada à imagem por um remap fixo (a câmera é presa ao carro).
        """
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        contrib = []
        for lamp in self._lamps:
            e = float(irradiance(np.array(x), np.array(y), lamp))
            contrib.append((e, lamp))
        total = self.cfg["ambient"] + sum(e for e, _ in contrib)
        contrib.sort(key=lambda t: -t[0])
        dark = np.zeros(grid_shape, np.float32)
        m = np.zeros(grid_shape, np.float32)
        x0, y0 = grid_origin
        sig_w = w_sum = 0.0
        for e, lamp in contrib[:max_lamps]:
            w = e / total
            if w < 0.04:
                continue
            dx, dy = lamp["x"] - x, lamp["y"] - y
            lamp_car = (c * dx + s * dy, -s * dx + c * dy, lamp["z"])
            poly = shadow_polygon(Occluder(body, 0.0, height), lamp_car)
            # grid: linha = frente (x0 → frente), coluna = esquerda (y0 → esquerda)
            px = np.column_stack([(poly[:, 1] - y0) / grid_res, (poly[:, 0] - x0) / grid_res])
            m[:] = 0.0
            cv2.fillPoly(m, [np.round(px * 16).astype(np.int32)], 1.0, cv2.LINE_AA, 4)
            dark += m * w
            sig_w += w * max(0.8, lamp["size"] * 0.15 / grid_res)
            w_sum += w
        if w_sum == 0.0:
            return dark
        # um desfoque só (penumbra média ponderada) em vez de um por lâmpada
        dark = cv2.GaussianBlur(dark, (0, 0), sig_w / w_sum)
        return np.clip(dark, 0.0, 0.95)


def occluders_from(scenario: Any, track_data: dict[str, Any]) -> list[Occluder]:
    """Tudo no cenário que fica entre as lâmpadas e o chão."""
    occ: list[Occluder] = []
    for b in scenario.buildings:
        occ.append(Occluder(np.asarray(b["footprint"], float), 0.0, b["height"]))
    for p in scenario.people:
        n = 12
        circ = np.array([[p["x"] + p["radius"] * math.cos(2 * math.pi * i / n),
                          p["y"] + p["radius"] * math.sin(2 * math.pi * i / n)] for i in range(n)])
        occ.append(Occluder(circ, 0.0, p["height"]))
    for sg in scenario.signs:
        f = sg["facing_yaw"]
        u = np.array([-math.sin(f), math.cos(f)]) * sg["diameter"] / 2
        c = np.array([sg["x"], sg["y"]])
        occ.append(Occluder(np.array([c - u, c + u, c + u * 1.001 + 0.002, c - u * 1.001 + 0.002]),
                            sg["bottom_height"], sg["bottom_height"] + sg["diameter"]))
    tl = track_data.get("traffic_light")
    if tl:
        f = tl["facing_yaw"]
        u = np.array([-math.sin(f), math.cos(f)]) * tl["box_width"] / 2
        n_ = np.array([math.cos(f), math.sin(f)]) * 0.05
        c = np.array([tl["x"], tl["y"]])
        zc = tl["yellow_center_height"]
        occ.append(Occluder(np.array([c - u - n_, c + u - n_, c + u + n_, c - u + n_]),
                            zc - tl["box_height"] / 2, zc + tl["box_height"] / 2))
    return occ
