"""Renderizador ``fast``: NumPy + OpenCV, só CPU, sem GPU nem ROS (CLAUDE.md §5A).

Por quadro:
1. **chão** — gira e translada a grade pré-calculada de "onde cada pixel toca
   o chão" (:meth:`FisheyeCamera.ground_lookup`) e amostra a textura com um
   único ``cv2.remap``. A distorção fisheye já está embutida na grade;
2. **sprites** — placas e semáforo como quadriláteros verticais projetados com
   a pose 3D real e ordenados por profundidade;
3. **pós** (opcional) — brilho, ruído, vinheta, desfoque de movimento.

Limitações conhecidas (aceitas no S1; o renderizador ``photo`` resolve):
* quarteirões e canteiros são planos — sem paralaxe dos 5 cm de altura nem
  oclusão de placas atrás deles;
* sprites usam homografia dos 4 cantos projetados: ignora a curvatura do
  fisheye DENTRO da placa (desprezível para 15 cm).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import cv2
import numpy as np

from .camera import FisheyeCamera
from .config import CameraParams
from .track import SIGN_CLASSES, Track, sign_texture

TL_STATES = ("vermelho", "amarelo", "verde", "apagado")


def traffic_light_texture(state: str, box_w: float, box_h: float, lens_d: float, spacing: float,
                          px_per_m: float = 1000.0) -> np.ndarray:
    """Caixa do semáforo em BGRA, 1 px/mm por padrão, uma cor acesa por vez."""
    W, H = int(round(box_w * px_per_m)), int(round(box_h * px_per_m))
    img = np.zeros((H, W, 4), np.uint8)
    img[:] = (28, 28, 30, 255)
    on = {"vermelho": (40, 40, 255), "amarelo": (30, 190, 255), "verde": (110, 230, 60)}
    off = {"vermelho": (30, 30, 70), "amarelo": (25, 60, 75), "verde": (35, 70, 30)}
    r = int(round(lens_d * px_per_m / 2))
    cx, cy = W // 2, H // 2
    for name, dy in (("vermelho", -spacing), ("amarelo", 0.0), ("verde", spacing)):
        col = on[name] if name == state else off[name]
        cv2.circle(img, (cx, int(round(cy + dy * px_per_m))), r, (*col, 255), -1, cv2.LINE_AA)
    return img


@dataclass
class Sprite:
    kind: str                       # "placa" | "semaforo" | "poste"
    label: str
    corners_world: np.ndarray       # (4, 3) TL, TR, BR, BL
    texture: np.ndarray | None      # BGRA; None = cor sólida
    color: tuple[int, int, int] = (40, 40, 40)
    meta: dict[str, Any] = field(default_factory=dict)


@dataclass
class PostFx:
    """Efeitos baratos para variar a aparência entre episódios."""

    brightness: float = 1.0         # multiplicador
    noise_std: float = 0.0          # desvio-padrão do ruído, em níveis de 0–255
    vignette: float = 0.0           # 0 = nenhuma, 0,5 = forte
    motion_blur_px: int = 0


class FastRenderer:
    def __init__(self, track: Track, cam_params: CameraParams, rng: np.random.Generator | None = None) -> None:
        self.track = track
        self.cam = FisheyeCamera(cam_params)
        self.p = cam_params
        lut = self.cam.ground_lookup()
        # O horizonte é fixo em relação à câmera: as linhas acima da primeira
        # que enxerga chão são só céu/parede, em QUALQUER pose. Elas não entram
        # no remap — copiamos o fundo pronto e pronto.
        rows = np.flatnonzero(lut.valid.any(axis=1))
        self._r0 = int(rows[0]) if rows.size else self.p.height
        fwd, left = lut.forward[self._r0:].copy(), lut.left[self._r0:].copy()
        # Abaixo de r0, os poucos pixels que não tocam o chão (perto do
        # horizonte curvo do fisheye) são mandados para longe da textura e
        # saem com a cor de borda = piso do ginásio.
        far = ~lut.valid[self._r0:]
        fwd[far], left[far] = 1.0e4, 0.0
        self._fwd, self._left = fwd, left
        self._tex = track.texture
        self._map_x = np.empty_like(self._fwd)
        self._map_y = np.empty_like(self._fwd)
        self._bg = self._make_background()
        self._out = self._bg.copy()
        self._vignette_cache: dict[float, np.ndarray] = {}
        self.rng = rng or np.random.default_rng()
        self.fx = PostFx()
        self._sign_tex = {c: sign_texture(c) for c in SIGN_CLASSES}
        self._sign_back = self._make_sign_back()
        self.tl_state = "vermelho"
        tl = track.traffic_light
        self._tl_tex = {
            s: traffic_light_texture(s, tl["box_width"], tl["box_height"], tl["lens_diameter"], tl["lens_spacing"])
            for s in TL_STATES
        } if tl else {}
        # A geometria dos sprites é estática: monta uma vez e empilha todos os
        # cantos num único array, para projetar tudo numa chamada só por quadro.
        self._sprites = self.sprites()
        self._corners = np.concatenate([sp.corners_world for sp in self._sprites]) if self._sprites else np.zeros((0, 3))
        self._centers = np.array([sp.corners_world[:, :2].mean(axis=0) for sp in self._sprites]).reshape(-1, 2)

    # ---------------- cenário ----------------

    def _make_background(self) -> np.ndarray:
        """Paredes do ginásio acima do horizonte: gradiente neutro."""
        H, W = self.p.height, self.p.width
        g = np.linspace(170, 120, H, dtype=np.float32)[:, None]
        bg = np.repeat(np.repeat(g, W, axis=1)[:, :, None], 3, axis=2)
        return bg.astype(np.uint8)

    def _make_sign_back(self) -> np.ndarray:
        t = np.zeros((64, 64, 4), np.uint8)
        cv2.circle(t, (32, 32), 30, (150, 150, 150, 255), -1, cv2.LINE_AA)
        return t

    def sprites(self) -> list[Sprite]:
        out: list[Sprite] = []
        pole_w = 0.02
        for sg in self.track.signs:
            f = sg["facing_yaw"]
            u = np.array([-math.sin(f), math.cos(f)])          # direita de quem olha a face
            c = np.array([sg["x"], sg["y"]])
            d, zb = sg["diameter"], sg["bottom_height"]
            out.append(Sprite("placa", sg["class"], _vquad(c, u, d, zb, zb + d), None,
                              meta={"slot": sg.get("slot"), "facing": f, "center": c}))
            out.append(Sprite("poste", "poste", _vquad(c - np.array([math.cos(f), math.sin(f)]) * 0.01,
                                                         u, pole_w, 0.0, zb), None, (70, 70, 72)))
        tl = self.track.traffic_light
        if tl:
            f = tl["facing_yaw"]
            u = np.array([-math.sin(f), math.cos(f)])
            c = np.array([tl["x"], tl["y"]])
            zc = tl["yellow_center_height"]
            zb, zt = zc - tl["box_height"] / 2, zc + tl["box_height"] / 2
            out.append(Sprite("semaforo", "semaforo", _vquad(c, u, tl["box_width"], zb, zt), None,
                              meta={"facing": f, "center": c}))
            out.append(Sprite("poste", "poste", _vquad(c - np.array([math.cos(f), math.sin(f)]) * 0.02,
                                                         u, 0.03, 0.0, zb), None, (90, 120, 160)))
        return out

    # ---------------- render ----------------

    def render(self, pose: tuple[float, float, float], tl_state: str | None = None,
               fx: PostFx | None = None) -> tuple[np.ndarray, dict[str, Any]]:
        x, y, yaw = pose
        t = self.track
        c, s = math.cos(yaw), math.sin(yaw)
        inv = 1.0 / t.res
        # Coordenadas na textura, sem alocar: map = A + fwd·B + left·C
        np.multiply(self._fwd, c * inv, out=self._map_x)
        self._map_x -= self._left * (s * inv)
        self._map_x += (x - t.x_min) * inv
        np.multiply(self._fwd, -s * inv, out=self._map_y)
        self._map_y -= self._left * (c * inv)
        self._map_y += (t.y_max - y) * inv

        # ``_out`` guarda o céu pronto nas linhas < r0; o remap só reescreve o
        # chão. Os sprites vão na CÓPIA, então ``_out`` nunca acumula sujeira.
        out = self._out
        cv2.remap(self._tex, self._map_x, self._map_y, cv2.INTER_LINEAR,
                  dst=out[self._r0:], borderMode=cv2.BORDER_CONSTANT, borderValue=(118, 122, 126))
        img = out.copy()                                      # o chamador pode guardar a imagem

        labels = self._draw_sprites(img, pose, tl_state or self.tl_state)
        self._post(img, fx or self.fx)
        return img, {"objects": labels}

    def _draw_sprites(self, img: np.ndarray, pose, tl_state: str) -> list[dict[str, Any]]:
        if not self._sprites:
            return []
        H, W = img.shape[:2]
        n = len(self._sprites)
        # Uma única projeção para todos os cantos de todos os sprites.
        cam_pts = self.cam.car_to_cam(self.cam.world_to_car(self._corners, pose))
        px, _ = self.cam.project_cam(cam_pts)
        depth = cam_pts[:, 2].reshape(n, 4)
        px = px.reshape(n, 4, 2)
        lo, hi = np.floor(px.min(axis=1)), np.ceil(px.max(axis=1))
        visible = (
            (depth.min(axis=1) > 0.05)                         # inteiro à frente da câmera
            & (hi[:, 0] >= 0) & (hi[:, 1] >= 0) & (lo[:, 0] < W) & (lo[:, 1] < H)
            & ((hi - lo).min(axis=1) >= 1)
        )
        idx = np.flatnonzero(visible)
        if idx.size == 0:
            return []
        cam_xy = self._camera_world_xy(pose)
        dist = np.hypot(*(self._centers[idx] - cam_xy).T)
        order = idx[np.argsort(-dist)]                         # longe → perto

        labels = []
        for i in order:
            sp = self._sprites[i]
            dist_i = float(np.hypot(*(self._centers[i] - cam_xy)))
            x0, y0 = lo[i].astype(int)
            x1, y1 = hi[i].astype(int)
            px_i = px[i]
            if sp.kind == "poste":
                cv2.fillConvexPoly(img, np.round(px_i).astype(np.int32), sp.color, cv2.LINE_AA)
                continue

            facing = sp.meta["facing"]
            to_cam = cam_xy - sp.meta["center"]
            front = (math.cos(facing) * to_cam[0] + math.sin(facing) * to_cam[1]) > 0
            if sp.kind == "placa":
                tex = self._sign_tex[sp.label] if front else self._sign_back
            else:
                tex = self._tl_tex[tl_state]
            _blit_quad(img, tex, px_i, (x0, y0, x1, y1))

            if front:
                bx0, by0 = max(x0, 0), max(y0, 0)
                bx1, by1 = min(x1, W - 1), min(y1, H - 1)
                labels.append({
                    "kind": sp.kind,
                    "class": sp.label if sp.kind == "placa" else tl_state,
                    "slot": sp.meta.get("slot"),
                    "bbox": [int(bx0), int(by0), int(bx1), int(by1)],
                    "bbox_px": [int(bx1 - bx0), int(by1 - by0)],
                    "truncated": bool(x0 < 0 or y0 < 0 or x1 >= W or y1 >= H),
                    "distance_m": round(dist_i, 3),
                })
        return labels

    def _camera_world_xy(self, pose) -> np.ndarray:
        x, y, yaw = pose
        cx, cy = self.p.x, self.p.y
        return np.array([x + cx * math.cos(yaw) - cy * math.sin(yaw),
                         y + cx * math.sin(yaw) + cy * math.cos(yaw)])

    def _post(self, img: np.ndarray, fx: PostFx) -> None:
        if fx.motion_blur_px > 1:
            k = np.zeros((fx.motion_blur_px, fx.motion_blur_px), np.float32)
            k[fx.motion_blur_px // 2, :] = 1.0 / fx.motion_blur_px
            cv2.filter2D(img, -1, k, dst=img)
        if fx.brightness != 1.0 or fx.noise_std > 0 or fx.vignette > 0:
            f = img.astype(np.float32)
            if fx.brightness != 1.0:
                f *= fx.brightness
            if fx.vignette > 0:
                f *= self._vignette(fx.vignette)[:, :, None]
            if fx.noise_std > 0:
                f += self.rng.normal(0.0, fx.noise_std, f.shape).astype(np.float32)
            np.clip(f, 0, 255, out=f)
            img[:] = f.astype(np.uint8)

    def _vignette(self, strength: float) -> np.ndarray:
        if strength not in self._vignette_cache:
            H, W = self.p.height, self.p.width
            yy, xx = np.mgrid[0:H, 0:W]
            r = np.hypot((xx - W / 2) / (W / 2), (yy - H / 2) / (H / 2)) / math.sqrt(2)
            self._vignette_cache[strength] = (1.0 - strength * r**2).astype(np.float32)
        return self._vignette_cache[strength]


def _vquad(center: np.ndarray, u: np.ndarray, width: float, z0: float, z1: float) -> np.ndarray:
    """Quadrilátero vertical: TL, TR, BR, BL (direita = +u)."""
    hl, hr = center - u * width / 2, center + u * width / 2
    return np.array([[hl[0], hl[1], z1], [hr[0], hr[1], z1], [hr[0], hr[1], z0], [hl[0], hl[1], z0]])


def _blit_quad(img: np.ndarray, tex: np.ndarray, quad_px: np.ndarray, bbox: tuple[int, int, int, int]) -> None:
    """Deforma a textura BGRA para o quadrilátero e mistura por alfa."""
    H, W = img.shape[:2]
    x0, y0, x1, y1 = bbox
    cx0, cy0, cx1, cy1 = max(x0, 0), max(y0, 0), min(x1, W), min(y1, H)
    if cx1 <= cx0 or cy1 <= cy0:
        return
    th, tw = tex.shape[:2]
    src = np.float32([[0, 0], [tw, 0], [tw, th], [0, th]])
    dst = (quad_px - [cx0, cy0]).astype(np.float32)
    M = cv2.getPerspectiveTransform(src, dst)
    patch = cv2.warpPerspective(tex, M, (cx1 - cx0, cy1 - cy0), flags=cv2.INTER_LINEAR,
                                borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0, 0))
    alpha = patch[:, :, 3:4].astype(np.float32) * (1.0 / 255.0)
    roi = img[cy0:cy1, cx0:cx1]
    roi[:] = (roi * (1.0 - alpha) + patch[:, :, :3] * alpha).astype(np.uint8)
