"""Carrega o mapa vetorial, rasteriza a textura do chão e responde consultas.

A textura do chão é desenhada UMA vez (2 mm/px por padrão, CLAUDE.md §5A) e
compartilhada entre todos os ambientes paralelos — só leitura.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .geometry import Polyline, point_in_polygon

# Cores em BGR (OpenCV). O renderizador de dataset (Blender) é que se
# preocupa com fotorrealismo; aqui basta o contraste certo.
COLORS = {
    "fora": (118, 122, 126),        # piso do ginásio fora da pista
    "carpete": (52, 52, 55),        # "carpete preto/cinza" (3.5.2)
    "linha": (232, 232, 232),       # "delimitação da rua em branco"
    "quadra": (52, 78, 118),        # marrom nas figuras
    "canteiro": (52, 96, 64),       # verde nas figuras
    "ilha": (200, 200, 205),
}


@dataclass
class Lane:
    id: str
    line: Polyline
    width: float


class Track:
    def __init__(self, data: dict[str, Any], resolution: float = 0.002, margin: float = 1.0) -> None:
        self.data = data
        b = data["bounds"]
        self.x_min, self.x_max = b["x_min"] - margin, b["x_max"] + margin
        self.y_min, self.y_max = b["y_min"] - margin, b["y_max"] + margin
        self.track_bounds = b
        self.res = resolution
        self.lanes = {
            ln["id"]: Lane(ln["id"], Polyline(np.asarray(ln["points"]), ln["closed"]), ln["width"])
            for ln in data["lanes"]
        }
        self.obstacles = [(o["kind"], np.asarray(o["polygon"], dtype=np.float64)) for o in data["obstacles"]]
        self.signs = data.get("signs", [])
        self.traffic_light = data.get("traffic_light")
        self.start = data["start"]
        self._texture: np.ndarray | None = None

    @classmethod
    def load(cls, path: Path | str, **kw: Any) -> Track:
        with open(path, encoding="utf-8") as fh:
            return cls(yaml.safe_load(fh), **kw)

    # ---------------- textura ----------------

    def to_px(self, xy: np.ndarray) -> np.ndarray:
        xy = np.asarray(xy, dtype=np.float64)
        u = (xy[..., 0] - self.x_min) / self.res
        v = (self.y_max - xy[..., 1]) / self.res
        return np.stack([u, v], axis=-1)

    @property
    def texture(self) -> np.ndarray:
        if self._texture is None:
            self._texture = self._rasterize()
        return self._texture

    def _rasterize(self) -> np.ndarray:
        import cv2

        W = int(math.ceil((self.x_max - self.x_min) / self.res))
        H = int(math.ceil((self.y_max - self.y_min) / self.res))
        img = np.empty((H, W, 3), dtype=np.uint8)
        img[:] = COLORS["fora"]

        shift = 4                                   # subpixel: coordenadas × 16
        def poly(pts: np.ndarray) -> np.ndarray:
            return np.round(self.to_px(pts) * (1 << shift)).astype(np.int32)

        # carpete: o contorno externo é a própria borda pintada
        border = next(ln for ln in self.data["lines"] if ln["kind"] == "borda")
        lw0 = border["width"]
        outer = np.asarray(border["points"]) * 1.0
        # expande meia espessura para o carpete ficar sob a linha inteira
        cx = (self.track_bounds["x_min"] + self.track_bounds["x_max"]) / 2
        cy = (self.track_bounds["y_min"] + self.track_bounds["y_max"]) / 2
        d = outer - [cx, cy]
        n = np.linalg.norm(d, axis=1, keepdims=True)
        outer = outer + d / np.maximum(n, 1e-9) * (lw0 / 2)
        cv2.fillPoly(img, [poly(outer)], COLORS["carpete"], cv2.LINE_AA, shift)

        for kind, pts in self.obstacles:
            cv2.fillPoly(img, [poly(pts)], COLORS[kind if kind in COLORS else "quadra"], cv2.LINE_AA, shift)

        for ln in self.data["lines"]:
            pts = np.asarray(ln["points"], dtype=np.float64)
            thick = max(1, int(round(ln["width"] / self.res)))
            if ln["style"] == "dashed":
                on, off = ln.get("dash", [0.3, 0.3])
                for seg in _dash(pts, ln["closed"], on, off):
                    cv2.polylines(img, [poly(seg)], False, COLORS["linha"], thick, cv2.LINE_AA, shift)
            else:
                cv2.polylines(img, [poly(pts)], ln["closed"], COLORS["linha"], thick, cv2.LINE_AA, shift)
        return img

    # ---------------- consultas ----------------

    def on_carpet(self, xy: np.ndarray) -> np.ndarray:
        """True onde o ponto está na rua (dentro da borda, fora de obstáculos)."""
        xy = np.atleast_2d(xy)
        b = self.track_bounds
        inside = (xy[:, 0] > b["x_min"]) & (xy[:, 0] < b["x_max"]) & (xy[:, 1] > b["y_min"]) & (xy[:, 1] < b["y_max"])
        for _, poly in self.obstacles:
            inside &= ~point_in_polygon(xy[:, 0], xy[:, 1], poly)
        return inside


def _dash(pts: np.ndarray, closed: bool, on: float, off: float) -> list[np.ndarray]:
    """Corta uma polilinha em traços de ``on`` m separados por ``off`` m."""
    if closed:
        pts = np.vstack([pts, pts[:1]])
    seg = np.diff(pts, axis=0)
    ln = np.hypot(seg[:, 0], seg[:, 1])
    s = np.concatenate([[0.0], np.cumsum(ln)])
    out = []
    t = 0.0
    while t < s[-1]:
        a, b = t, min(t + on, s[-1])
        ts = np.concatenate([[a], s[(s > a) & (s < b)], [b]])
        out.append(np.stack([np.interp(ts, s, pts[:, 0]), np.interp(ts, s, pts[:, 1])], axis=1))
        t += on + off
    return out


# --------------------------------------------------------------------------
# Texturas das placas (Figura 8 do regulamento)
# --------------------------------------------------------------------------

SIGN_CLASSES = (
    "seguir_em_frente",
    "virar_esquerda",
    "virar_direita",
    "proibido_seguir_em_frente",
    "proibido_virar_esquerda",
    "proibido_virar_direita",
)
"""As 6 placas da Figura 8 (detalhe) do regulamento. O simulador do
organizador só tem as 3 primeiras."""


def sign_texture(cls: str, size: int = 160) -> np.ndarray:
    """Desenha a placa em BGRA. Aproximação — para TREINO use também fotos
    reais tiradas do vídeo oficial (https://youtu.be/YAg2p1kShZA)."""
    import cv2

    if cls not in SIGN_CLASSES:
        raise ValueError(f"classe de placa desconhecida: {cls!r}")
    img = np.zeros((size, size, 4), dtype=np.uint8)
    c = size // 2
    R = size // 2 - 2
    red = (30, 30, 215, 255)
    cv2.circle(img, (c, c), R, red, -1, cv2.LINE_AA)
    cv2.circle(img, (c, c), int(R * 0.80), (245, 245, 245, 255), -1, cv2.LINE_AA)

    k = size / 160.0
    black = (20, 20, 20, 255)
    th = max(2, int(13 * k))
    base = cls.replace("proibido_", "")
    if base == "seguir_em_frente":
        cv2.line(img, (c, int(c + 45 * k)), (c, int(c - 22 * k)), black, th, cv2.LINE_AA)
        head = np.array([[c, c - 52 * k], [c - 22 * k, c - 18 * k], [c + 22 * k, c - 18 * k]], np.int32)
        cv2.fillPoly(img, [head], black, cv2.LINE_AA)
    else:
        # Haste sobe e faz um quarto de círculo para o lado da curva.
        # sgn = −1 vira à esquerda, +1 à direita. A haste fica deslocada para
        # o lado oposto, para a curva e a ponta caberem no círculo branco.
        sgn = -1 if base == "virar_esquerda" else 1
        x0 = c - sgn * 14 * k
        y_top = c + 2 * k
        r_arc = 24 * k
        pts = [(x0, c + 45 * k)]
        for i in range(13):
            phi = math.pi / 2 * i / 12
            pts.append((x0 + sgn * r_arc * (1 - math.cos(phi)), y_top - r_arc * math.sin(phi)))
        cv2.polylines(img, [np.round(np.array(pts)).astype(np.int32)], False, black, th, cv2.LINE_AA)
        end_x, end_y = pts[-1]
        head = np.array([[end_x + sgn * 24 * k, end_y],
                         [end_x, end_y - 20 * k],
                         [end_x, end_y + 20 * k]])
        cv2.fillPoly(img, [np.round(head).astype(np.int32)], black, cv2.LINE_AA)
    if cls.startswith("proibido_"):
        # barra diagonal vermelha, do alto à esquerda para baixo à direita (Fig. 8)
        a = int(R * 0.80 * 0.707)
        cv2.line(img, (c - a, c - a), (c + a, c + a), red, max(3, int(16 * k)), cv2.LINE_AA)
    return img
