"""Renderizador ``fast``: NumPy + OpenCV, só CPU, sem GPU nem ROS (CLAUDE.md §5A).

Por quadro:
1. **chão** — gira e translada a grade pré-calculada de "onde cada pixel toca
   o chão" e amostra a textura com um único ``cv2.remap`` (o fisheye já está
   embutido na grade);
2. **luz** — amostra o mapa de luz do cenário com o mesmo remap (escala 2 mm →
   2 cm) e multiplica; soma a sombra do próprio carro, que anda com ele;
3. **objetos** — placas, semáforo, postes, prédios (maquetes) e público,
   projetados com a pose 3D real e desenhados do mais longe para o mais perto;
4. **pós** — ruído, vinheta, desfoque de movimento (opcionais).

Rótulos: cada placa/semáforo visível sai com caixa, distância e a FRAÇÃO
VISÍVEL — quanto dela não está escondida atrás de prédio ou pessoa. Uma placa
escondida não deve virar exemplo de treino como se estivesse à vista.

Limitações aceitas no ``fast`` (o ``photo`` resolve):
* objetos com textura usam homografia dos 4 cantos (ignora a curvatura do
  fisheye DENTRO de uma placa de 15 cm — desprezível);
* ordenação por distância do centro (pintor), não z-buffer;
* quarteirões e canteiros continuam planos (5 cm).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import cv2
import numpy as np

from .camera import FisheyeCamera
from .config import CameraParams
from .lighting import LIGHT_ONE, LightField, occluders_from
from .track import SIGN_CLASSES, Track, sign_texture

TL_STATES = ("vermelho", "amarelo", "verde", "apagado")
NEAR = 0.04                       # plano próximo da câmera (m)
FACE_SEG = 8                      # subdivisões por aresta de prédio


def traffic_light_textures(box_w: float, box_h: float, lens_d: float, spacing: float,
                           px_per_m: float = 1000.0) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """Caixa com as lentes apagadas (recebe luz do ambiente) e, por estado,
    a lente acesa (EMISSIVA: não escurece na sombra)."""
    W, H = int(round(box_w * px_per_m)), int(round(box_h * px_per_m))
    r = int(round(lens_d * px_per_m / 2))
    cx, cy = W // 2, H // 2
    on = {"vermelho": (40, 40, 255), "amarelo": (30, 190, 255), "verde": (110, 230, 60)}
    off = {"vermelho": (30, 30, 70), "amarelo": (25, 60, 75), "verde": (35, 70, 30)}
    pos = {"vermelho": -spacing, "amarelo": 0.0, "verde": spacing}
    base = np.zeros((H, W, 4), np.uint8)
    base[:] = (28, 28, 30, 255)
    for name, dy in pos.items():
        cv2.circle(base, (cx, int(round(cy + dy * px_per_m))), r, (*off[name], 255), -1, cv2.LINE_AA)
    emissive = {}
    for name, dy in pos.items():
        e = np.zeros((H, W, 4), np.uint8)
        cv2.circle(e, (cx, int(round(cy + dy * px_per_m))), r, (*on[name], 255), -1, cv2.LINE_AA)
        emissive[name] = e
    emissive["apagado"] = np.zeros((H, W, 4), np.uint8)
    return base, emissive


def person_texture(shirt, pants, w: int = 48, h: int = 200) -> np.ndarray:
    t = np.zeros((h, w, 4), np.uint8)
    leg = int(h * 0.47)
    cv2.rectangle(t, (int(w * 0.18), h - leg), (int(w * 0.46), h - 1), (*pants, 255), -1)
    cv2.rectangle(t, (int(w * 0.54), h - leg), (int(w * 0.82), h - 1), (*pants, 255), -1)
    cv2.rectangle(t, (int(w * 0.08), int(h * 0.17)), (int(w * 0.92), h - leg), (*shirt, 255), -1)
    cv2.circle(t, (w // 2, int(h * 0.09)), int(w * 0.24), (120, 150, 190, 255), -1, cv2.LINE_AA)
    return t


@dataclass
class Sprite:
    kind: str                       # placa | semaforo | poste | pessoa | predio
    label: str
    corners_world: np.ndarray       # (4, 3) TL, TR, BR, BL — ou polígono (N, 3) se predio
    texture: np.ndarray | None = None
    color: tuple[int, int, int] = (40, 40, 40)
    meta: dict[str, Any] = field(default_factory=dict)


@dataclass
class PostFx:
    """Efeitos baratos para variar a aparência entre episódios."""

    brightness: float = 1.0
    noise_std: float = 0.0
    vignette: float = 0.0
    motion_blur_px: int = 0


class FastRenderer:
    def __init__(self, track: Track, cam_params: CameraParams, rng: np.random.Generator | None = None,
                 car_body: tuple[np.ndarray, float] | None = None) -> None:
        self.track = track
        self.cam = FisheyeCamera(cam_params)
        self.p = cam_params
        lut = self.cam.ground_lookup()
        # Linhas acima do horizonte (fixo em relação à câmera) não entram no remap.
        rows = np.flatnonzero(lut.valid.any(axis=1))
        self._r0 = int(rows[0]) if rows.size else self.p.height
        fwd, left = lut.forward[self._r0:].copy(), lut.left[self._r0:].copy()
        far = ~lut.valid[self._r0:]
        fwd[far], left[far] = 1.0e4, 0.0
        self._fwd, self._left = fwd, left
        self._map_x = np.empty_like(fwd)
        self._map_y = np.empty_like(fwd)
        # Grade da sombra do carro (referencial do carro, 2 cm/px) e o remap FIXO
        # que a leva à imagem — a câmera é presa ao carro.
        # 2 m à frente × 2 m de lado: a sombra de um carro de 17 cm sob lâmpadas
        # a ≥ 3 m não passa disso.
        self._cs_res, self._cs_origin, self._cs_shape = 0.02, (-0.6, -1.0), (100, 100)
        self._cs_mx = ((left - self._cs_origin[1]) / self._cs_res).astype(np.float32)
        self._cs_my = ((fwd - self._cs_origin[0]) / self._cs_res).astype(np.float32)
        self._car_body = car_body                       # (polígono no ref. do carro, altura)

        self._bg0 = self._make_background()
        self._bg = self._bg0
        self._out = self._bg.copy()
        self._vignette_cache: dict[float, np.ndarray] = {}
        self.rng = rng or np.random.default_rng()
        self.fx = PostFx()
        self._sign_tex = {c: sign_texture(c) for c in SIGN_CLASSES}
        self._sign_back = self._make_sign_back()
        self.tl_state = "vermelho"
        tl = track.traffic_light
        self._tl_base, self._tl_emissive = traffic_light_textures(
            tl["box_width"], tl["box_height"], tl["lens_diameter"], tl["lens_spacing"]) if tl else (None, {})
        self.light: LightField | None = None
        self.scenario = None
        self.set_scenario(None)

    # ---------------- cenário ----------------

    def set_scenario(self, scenario, lighting: bool = True) -> None:
        """Troca placas, zebras, prédios, público e luz. Custa ~0,1–0,3 s."""
        self.scenario = scenario
        t = self.track
        if scenario is None:
            self._tex = t.texture
            t.signs = []
            self.light = None
            self._bg = self._bg0
        else:
            self._tex = t.texture_with(scenario.crosswalks)
            t.signs = scenario.signs
            if lighting:
                self.light = LightField(scenario.lighting, occluders_from(scenario, t.data),
                                        t.x_min, t.x_max, t.y_min, t.y_max, t.track_bounds)
                # A luz do ambiente não muda durante o episódio: entra na
                # textura UMA vez (~0,3 s) em vez de um remap + produto por quadro.
                H, W = self._tex.shape[:2]
                lm = cv2.resize(self.light.map, (W, H), interpolation=cv2.INTER_LINEAR)
                cv2.multiply(self._tex, lm, dst=self._tex, scale=1.0 / LIGHT_ONE)
                del lm
                k = min(1.25, max(0.45, self.light.mean_level))
                self._bg = np.clip(self._bg0.astype(np.float32) * k, 0, 255).astype(np.uint8)
            else:
                self.light = None
                self._bg = self._bg0
        self._out = self._bg.copy()
        self._static = self._build_static()
        self._people = []
        if scenario is not None:
            for i, p in enumerate(scenario.people):
                self._people.append((p, person_texture(p["shirt"], p["pants"]), i))
        pe = scenario.people if scenario is not None else []
        self._people_arr = (np.array([[p["x"], p["y"]] for p in pe]).reshape(-1, 2),
                            np.array([p["radius"] for p in pe]), np.array([p["height"] for p in pe]))
        self._people_light = [self._light_at((p["x"], p["y"])) if self.light is not None else None for p in pe]
        self._cs_cache: tuple[tuple[float, float, float], np.ndarray | None] | None = None

    def _make_background(self) -> np.ndarray:
        H, W = self.p.height, self.p.width
        g = np.linspace(170, 120, H, dtype=np.float32)[:, None]
        return np.repeat(np.repeat(g, W, axis=1)[:, :, None], 3, axis=2).astype(np.uint8)

    def _make_sign_back(self) -> np.ndarray:
        t = np.zeros((64, 64, 4), np.uint8)
        cv2.circle(t, (32, 32), 30, (150, 150, 150, 255), -1, cv2.LINE_AA)
        return t

    def _build_static(self) -> dict[str, Any]:
        """Objetos que não mudam com a pose: projetados juntos, numa chamada."""
        quads: list[Sprite] = []
        for sg in self.track.signs:
            f = sg["facing_yaw"]
            u = np.array([-math.sin(f), math.cos(f)])
            c = np.array([sg["x"], sg["y"]])
            d, zb = sg["diameter"], sg["bottom_height"]
            quads.append(Sprite("placa", sg["class"], _vquad(c, u, d, zb, zb + d),
                                meta={"facing": f, "center": c, "node": sg.get("node"),
                                      "arrival": sg.get("arrival"), "on_route": sg.get("on_route")}))
            quads.append(Sprite("poste", "poste", _vquad(c - np.array([math.cos(f), math.sin(f)]) * 0.01,
                                                          u, 0.02, 0.0, zb), color=(70, 70, 72),
                                meta={"center": c}))
        tl = self.track.traffic_light
        if tl:
            f = tl["facing_yaw"]
            u = np.array([-math.sin(f), math.cos(f)])
            c = np.array([tl["x"], tl["y"]])
            zc = tl["yellow_center_height"]
            quads.append(Sprite("semaforo", "semaforo", _vquad(c, u, tl["box_width"], zc - tl["box_height"] / 2,
                                                                zc + tl["box_height"] / 2),
                                meta={"facing": f, "center": c}))
            quads.append(Sprite("poste", "poste", _vquad(c - np.array([math.cos(f), math.sin(f)]) * 0.02,
                                                          u, 0.03, 0.0, zc - tl["box_height"] / 2),
                                color=(90, 120, 160), meta={"center": c}))
        faces: list[Sprite] = []
        if self.scenario is not None:
            for bi, b in enumerate(self.scenario.buildings):
                fp, h = np.asarray(b["footprint"], float), b["height"]
                ctr = fp.mean(axis=0)
                n = len(fp)
                for i in range(n):
                    p0, p1 = fp[i], fp[(i + 1) % n]
                    edge = p1 - p0
                    normal = np.array([edge[1], -edge[0]]) / max(np.hypot(*edge), 1e-9)   # para fora (CCW)
                    poly = np.array([[p0[0], p0[1], h], [p1[0], p1[1], h], [p1[0], p1[1], 0.0], [p0[0], p0[1], 0.0]])
                    faces.append(Sprite("predio", f"predio_{bi}", poly, color=b["color"],
                                        meta={"normal": normal, "center": (p0 + p1) / 2, "bctr": ctr}))
                if n == 4:                                  # telhado (retângulo)
                    roof = np.column_stack([fp, np.full(n, h)])
                    faces.append(Sprite("predio", f"predio_{bi}", roof, color=b["color"],
                                        meta={"normal": np.zeros(2), "center": ctr, "bctr": ctr, "roof": True}))
        corners = np.concatenate([q.corners_world for q in quads]) if quads else np.zeros((0, 3))
        centers = np.array([q.meta["center"] for q in quads]).reshape(-1, 2)
        # A luz é estática no episódio: a cor de cada objeto sai daqui, uma vez.
        for q in quads:
            q.meta["light"] = self._light_at(q.meta["center"]) if self.light is not None else None
            if q.kind == "poste":
                L = q.meta["light"] if q.meta["light"] is not None else np.ones(3)
                q.meta["fill"] = tuple(int(min(255, ci * L[k])) for k, ci in enumerate(q.color))
        dom: dict[int, np.ndarray | None] = {}
        for f in faces:
            L = self._light_at(f.meta["bctr"])
            shade = 1.0 if f.meta.get("roof") else 0.55
            if not f.meta.get("roof"):
                key = id(f.meta["bctr"])
                if key not in dom:
                    dom[key] = self._dominant_dir(f.meta["bctr"])
                if dom[key] is not None:
                    shade += 0.45 * max(0.0, float(np.dot(f.meta["normal"], dom[key])))
            f.meta["fill"] = tuple(int(min(255, ci * L[k] * shade)) for k, ci in enumerate(f.color))
        # Faces dos prédios com as arestas já subdivididas NO MUNDO (reta no
        # mundo vira curva no fisheye): uma projeção só, para todas, por quadro.
        F = len(faces)
        dense = np.zeros((F, 4 * FACE_SEG, 3))
        k = (np.arange(FACE_SEG) / FACE_SEG)[:, None]
        for i, f in enumerate(faces):
            c4 = f.corners_world
            dense[i] = np.concatenate([c4[j] + (c4[(j + 1) % 4] - c4[j]) * k for j in range(4)])
        face_arr = {
            "dense": dense,
            "normal": np.array([f.meta["normal"] for f in faces]).reshape(F, 2),
            "center": np.array([f.meta["center"] for f in faces]).reshape(F, 2),
            "roof": np.array([bool(f.meta.get("roof")) for f in faces], bool),
            "z": np.array([f.corners_world[0, 2] for f in faces]),
        }
        return {"quads": quads, "corners": corners, "centers": centers, "faces": faces, "face_arr": face_arr}

    # ---------------- render ----------------

    def render(self, pose: tuple[float, float, float], tl_state: str | None = None,
               fx: PostFx | None = None) -> tuple[np.ndarray, dict[str, Any]]:
        x, y, yaw = pose
        t = self.track
        c, s = math.cos(yaw), math.sin(yaw)
        inv = 1.0 / t.res
        np.multiply(self._fwd, c * inv, out=self._map_x)
        self._map_x -= self._left * (s * inv)
        self._map_x += (x - t.x_min) * inv
        np.multiply(self._fwd, -s * inv, out=self._map_y)
        self._map_y -= self._left * (c * inv)
        self._map_y += (t.y_max - y) * inv

        out = self._out
        cv2.remap(self._tex, self._map_x, self._map_y, cv2.INTER_LINEAR,
                  dst=out[self._r0:], borderMode=cv2.BORDER_CONSTANT, borderValue=(118, 122, 126))
        img = out.copy()

        if self.light is not None and self._car_body is not None:
            # Luz do ambiente já está na textura. Falta a sombra do PRÓPRIO
            # carro, que anda com ele. No referencial do carro ela só muda
            # quando a direção das lâmpadas muda — recalcula a cada 4 cm / 2°.
            cc = self._cs_cache
            if cc is None or math.hypot(x - cc[0][0], y - cc[0][1]) > 0.04 \
                    or abs(math.remainder(yaw - cc[0][2], 2 * math.pi)) > 0.035:
                dark = self.light.car_shadow(pose, self._car_body[0], self._car_body[1],
                                             self._cs_origin, self._cs_res, self._cs_shape)
                k3 = None
                if dark.max() > 0.01:
                    keep = cv2.remap(1.0 - dark, self._cs_mx, self._cs_my, cv2.INTER_LINEAR,
                                     borderMode=cv2.BORDER_CONSTANT, borderValue=1.0)
                    k8 = (keep * 255).astype(np.uint8)
                    k3 = cv2.merge([k8, k8, k8])
                self._cs_cache = cc = ((x, y, yaw), k3)
            if cc[1] is not None:
                ground = img[self._r0:]
                cv2.multiply(ground, cc[1], dst=ground, scale=1.0 / 255)

        labels = self._draw_objects(img, pose, tl_state or self.tl_state)
        self._post(img, fx or self.fx)
        return img, {"objects": labels}

    # ---------------- objetos ----------------

    def _light_at(self, xy) -> np.ndarray:
        return self.light.at(float(xy[0]), float(xy[1])) if self.light is not None else np.ones(3, np.float32)

    def _dominant_dir(self, xy) -> np.ndarray | None:
        if self.light is None or not self.scenario.lighting["lamps"]:
            return None
        best = max(self.scenario.lighting["lamps"],
                   key=lambda L: L["intensity"] * L["z"] ** 3 / ((L["z"] ** 2 + (L["x"] - xy[0]) ** 2 + (L["y"] - xy[1]) ** 2) ** 1.5))
        d = np.array([best["x"] - xy[0], best["y"] - xy[1]])
        return d / max(np.hypot(*d), 1e-9)

    def _draw_objects(self, img: np.ndarray, pose, tl_state: str) -> list[dict[str, Any]]:
        H, W = img.shape[:2]
        cam_xy = self._camera_world_xy(pose)
        items: list[tuple[float, str, Any]] = []      # (distância, tipo, dados)
        st = self._static

        # objetos estáticos com 4 cantos: uma projeção para todos
        n = len(st["quads"])
        if n:
            cam_pts = self.cam.car_to_cam(self.cam.world_to_car(st["corners"], pose))
            px, _ = self.cam.project_cam(cam_pts)
            depth = cam_pts[:, 2].reshape(n, 4)
            px = px.reshape(n, 4, 2)
            lo, hi = np.floor(px.min(axis=1)), np.ceil(px.max(axis=1))
            vis = ((depth.min(axis=1) > 0.05) & (hi[:, 0] >= 0) & (hi[:, 1] >= 0)
                   & (lo[:, 0] < W) & (lo[:, 1] < H) & ((hi - lo).min(axis=1) >= 1))
            for i in np.flatnonzero(vis):
                d = float(np.hypot(*(st["centers"][i] - cam_xy)))
                items.append((d, "quad", (st["quads"][i], px[i], lo[i], hi[i])))

        # prédios: todas as faces numa projeção só; recorte no plano próximo
        # só para as que atravessam a câmera (parede ao lado do carro)
        fa = st["face_arr"]
        if len(st["faces"]):
            rel = cam_xy - fa["center"]
            front = np.where(fa["roof"], fa["z"] < self.p.z, np.einsum("ij,ij->i", fa["normal"], rel) > 0)
            idx = np.flatnonzero(front)
            if idx.size:
                pts = fa["dense"][idx].reshape(-1, 3)
                cam = self.cam.car_to_cam(self.cam.world_to_car(pts, pose)).reshape(idx.size, -1, 3)
                zmin, zmax = cam[:, :, 2].min(axis=1), cam[:, :, 2].max(axis=1)
                ok = zmin >= NEAR
                if ok.any():
                    px, _ = self.cam.project_cam(cam[ok].reshape(-1, 3))
                    px = px.reshape(int(ok.sum()), -1, 2)
                    lo, hi = px.min(axis=1), px.max(axis=1)
                    on = (hi[:, 0] >= 0) & (hi[:, 1] >= 0) & (lo[:, 0] < W) & (lo[:, 1] < H)
                    for j, fi in enumerate(idx[ok]):
                        if on[j]:
                            f = st["faces"][fi]
                            items.append((float(np.hypot(*rel[fi])), "face", (f, px[j])))
                for fi in idx[(~ok) & (zmax > NEAR)]:
                    f = st["faces"][fi]
                    poly = self._project_polygon(f.corners_world, pose)
                    if poly is not None:
                        items.append((float(np.hypot(*rel[fi])), "face", (f, poly)))

        # público: billboards virados para a câmera, todos numa projeção só
        if self._people:
            P, R, Hh = self._people_arr
            to = P - cam_xy
            d = np.hypot(to[:, 0], to[:, 1])
            u = np.column_stack([to[:, 1], -to[:, 0]]) / np.maximum(d, 1e-9)[:, None]   # direita de quem olha
            hl, hr = P - u * R[:, None], P + u * R[:, None]
            z0, z1 = np.zeros_like(R), Hh
            corners = np.stack([np.column_stack([hl, z1]), np.column_stack([hr, z1]),
                                np.column_stack([hr, z0]), np.column_stack([hl, z0])], axis=1)
            cam_pts = self.cam.car_to_cam(self.cam.world_to_car(corners.reshape(-1, 3), pose)).reshape(-1, 4, 3)
            okp = (d >= 0.3) & (cam_pts[:, :, 2].min(axis=1) > 0.05)
            if okp.any():
                ppx_all, _ = self.cam.project_cam(cam_pts[okp].reshape(-1, 3))
                ppx_all = ppx_all.reshape(-1, 4, 2)
                for j, i in enumerate(np.flatnonzero(okp)):
                    ppx = ppx_all[j]
                    lo, hi = np.floor(ppx.min(axis=0)), np.ceil(ppx.max(axis=0))
                    if hi[0] < 0 or hi[1] < 0 or lo[0] >= W or lo[1] >= H:
                        continue
                    _, tex, _ = self._people[i]
                    items.append((float(d[i]), "pessoa", (i, tex, ppx, lo, hi)))

        items.sort(key=lambda it: -it[0])                  # pintor: longe → perto
        labels: list[dict[str, Any]] = []
        drawn: list[tuple[float, np.ndarray]] = []         # (distância, polígono) dos que escondem

        for dist, kind, data in items:
            if kind == "face":
                f, poly = data
                cv2.fillPoly(img, [np.round(poly).astype(np.int32)], f.meta["fill"], cv2.LINE_AA)
                drawn.append((dist, poly))
            elif kind == "pessoa":
                i, tex, ppx, lo, hi = data
                _blit_quad(img, tex, ppx, (int(lo[0]), int(lo[1]), int(hi[0]), int(hi[1])),
                           light=self._people_light[i])
                drawn.append((dist, ppx))
            else:
                sp, qpx, lo, hi = data
                x0, y0 = int(lo[0]), int(lo[1])
                x1, y1 = int(hi[0]), int(hi[1])
                L = sp.meta["light"]
                if sp.kind == "poste":
                    cv2.fillConvexPoly(img, np.round(qpx).astype(np.int32), sp.meta["fill"], cv2.LINE_AA)
                    continue
                facing = sp.meta["facing"]
                to_cam = cam_xy - sp.meta["center"]
                front = (math.cos(facing) * to_cam[0] + math.sin(facing) * to_cam[1]) > 0
                if sp.kind == "placa":
                    _blit_quad(img, self._sign_tex[sp.label] if front else self._sign_back, qpx,
                               (x0, y0, x1, y1), light=L)
                else:
                    _blit_quad(img, self._tl_base, qpx, (x0, y0, x1, y1), light=L)
                    if front:
                        _blit_quad(img, self._tl_emissive[tl_state], qpx, (x0, y0, x1, y1))
                    drawn.append((dist, qpx))
                if front:
                    bx0, by0 = max(x0, 0), max(y0, 0)
                    bx1, by1 = min(x1, W - 1), min(y1, H - 1)
                    labels.append({
                        "kind": sp.kind,
                        "class": sp.label if sp.kind == "placa" else tl_state,
                        "node": sp.meta.get("node"), "arrival": sp.meta.get("arrival"),
                        "on_route": sp.meta.get("on_route"),
                        "bbox": [bx0, by0, bx1, by1], "bbox_px": [bx1 - bx0, by1 - by0],
                        "truncated": bool(x0 < 0 or y0 < 0 or x1 >= W or y1 >= H),
                        "distance_m": round(dist, 3),
                        "_quad": qpx, "_dist": dist,
                    })

        for lb in labels:                                   # quanto da placa ficou à vista
            lb["visible_frac"] = round(_visible_fraction(lb.pop("_quad"), lb.pop("_dist"), drawn, W, H), 3)
        return labels

    def _project_polygon(self, poly_world: np.ndarray, pose, seg: int = FACE_SEG) -> np.ndarray | None:
        """Projeta um polígono 3D com recorte no plano próximo e arestas
        subdivididas (no fisheye, reta longa vira curva)."""
        cam = self.cam.car_to_cam(self.cam.world_to_car(poly_world, pose))
        cam = _clip_near(cam, NEAR)
        if cam is None:
            return None
        dense = []
        m = len(cam)
        for i in range(m):
            a, b = cam[i], cam[(i + 1) % m]
            for k in range(seg):
                dense.append(a + (b - a) * (k / seg))
        px, _ = self.cam.project_cam(np.asarray(dense))
        if not np.isfinite(px).all():
            return None
        W, H = self.p.width, self.p.height
        if px[:, 0].max() < 0 or px[:, 1].max() < 0 or px[:, 0].min() >= W or px[:, 1].min() >= H:
            return None
        return np.clip(px, -4 * W, 5 * W)

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


# --------------------------------------------------------------------------
# auxiliares
# --------------------------------------------------------------------------


def _vquad(center: np.ndarray, u: np.ndarray, width: float, z0: float, z1: float) -> np.ndarray:
    """Quadrilátero vertical: TL, TR, BR, BL (direita = +u)."""
    hl, hr = center - u * width / 2, center + u * width / 2
    return np.array([[hl[0], hl[1], z1], [hr[0], hr[1], z1], [hr[0], hr[1], z0], [hl[0], hl[1], z0]])


def _clip_near(poly: np.ndarray, near: float) -> np.ndarray | None:
    """Sutherland–Hodgman contra o plano z = near (referencial da câmera)."""
    out = []
    m = len(poly)
    for i in range(m):
        a, b = poly[i], poly[(i + 1) % m]
        ina, inb = a[2] >= near, b[2] >= near
        if ina:
            out.append(a)
        if ina != inb:
            t = (near - a[2]) / (b[2] - a[2])
            out.append(a + (b - a) * t)
    return np.asarray(out) if len(out) >= 3 else None


def _blit_quad(img: np.ndarray, tex: np.ndarray, quad_px: np.ndarray, bbox: tuple[int, int, int, int],
               light: np.ndarray | None = None) -> None:
    """Deforma a textura BGRA para o quadrilátero e mistura por alfa.
    ``light`` (BGR, 1,0 = normal) escurece/tinge a textura — exceto emissivas."""
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
    rgb = patch[:, :, :3].astype(np.float32)
    if light is not None:
        rgb *= np.asarray(light, np.float32)
    roi = img[cy0:cy1, cx0:cx1]
    roi[:] = np.clip(roi * (1.0 - alpha) + rgb * alpha, 0, 255).astype(np.uint8)


def _visible_fraction(quad: np.ndarray, dist: float, drawn: list[tuple[float, np.ndarray]], W: int, H: int) -> float:
    """Fração da placa que não ficou atrás de algo MAIS PERTO (prédio, pessoa)."""
    x0, y0 = np.floor(quad.min(axis=0)).astype(int)
    x1, y1 = np.ceil(quad.max(axis=0)).astype(int)
    x0, y0, x1, y1 = max(x0, 0), max(y0, 0), min(x1, W), min(y1, H)
    if x1 <= x0 or y1 <= y0:
        return 0.0
    own = np.zeros((y1 - y0, x1 - x0), np.uint8)
    cv2.fillPoly(own, [np.round(quad - [x0, y0]).astype(np.int32)], 1)
    area = int(own.sum())
    if area == 0:
        return 0.0
    hide = np.zeros_like(own)
    for d, poly in drawn:
        if d < dist:
            cv2.fillPoly(hide, [np.round(poly - [x0, y0]).astype(np.int32)], 1)
    return 1.0 - float((own & hide).sum()) / area
