"""Modelo de câmera fisheye e projeção do chão.

Implementa o modelo Kannala-Brandt — o mesmo do ``cv2.fisheye`` — em NumPy:

    θ   = atan(r),   r = ‖(X/Z, Y/Z)‖
    θ_d = θ (1 + k1 θ² + k2 θ⁴ + k3 θ⁶ + k4 θ⁸)
    u   = fx · θ_d · (X/Z) / r + cx

Escrever o modelo à mão (em vez de chamar o OpenCV) tem dois motivos:
1. permite pré-calcular, UMA vez, o raio de cada pixel — o truque que faz o
   renderizador ``fast`` passar de 1.000 passos/s;
2. a geometria fica testável sem OpenCV. Um teste confere, onde o OpenCV
   existe, que este código e o ``cv2.fisheye.projectPoints`` dão o mesmo pixel.

Referenciais:
* câmera: x direita, y baixo, z frente (convenção OpenCV);
* carro (base_link): x frente, y esquerda, z cima.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .config import CameraParams


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


#: Eixos da câmera expressos no carro, com pitch/roll/yaw zero:
#: z_cam → +x_carro (frente), x_cam → −y_carro (direita), y_cam → −z_carro (baixo).
_R0 = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float64)


@dataclass
class GroundLookup:
    """Para cada pixel, onde o raio toca o chão, no referencial do carro."""

    forward: np.ndarray     # (H, W) float32, metros à frente de base_link
    left: np.ndarray        # (H, W) float32, metros à esquerda
    valid: np.ndarray       # (H, W) bool: o raio aponta para o chão e acerta perto


class FisheyeCamera:
    def __init__(self, params: CameraParams) -> None:
        self.p = params
        self.K = np.array([[params.fx, 0, params.cx], [0, params.fy, params.cy], [0, 0, 1]])
        self.dist = np.array(params.dist, dtype=np.float64)
        # cam → carro
        self.R_car_cam = _rz(params.yaw) @ _ry(params.pitch) @ _rx(params.roll) @ _R0
        self.t_car_cam = np.array([params.x, params.y, params.z])

    # ---------------- modelo óptico ----------------

    def _distort_theta(self, theta: np.ndarray) -> np.ndarray:
        k1, k2, k3, k4 = self.dist
        t2 = theta * theta
        return theta * (1 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4))))

    def _undistort_theta(self, theta_d: np.ndarray, iters: int = 8) -> np.ndarray:
        """Inverte θ_d(θ) por Newton. Com dist = 0, θ = θ_d exatamente."""
        if not np.any(self.dist):
            return theta_d.copy()
        k1, k2, k3, k4 = self.dist
        theta = theta_d.copy()
        for _ in range(iters):
            t2 = theta * theta
            f = theta * (1 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4)))) - theta_d
            df = 1 + t2 * (3 * k1 + t2 * (5 * k2 + t2 * (7 * k3 + 9 * k4 * t2)))
            theta -= f / df
        return theta

    def project_cam(self, pts_cam: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Projeta pontos 3D (N,3) no referencial da câmera → (pixels (N,2), visível (N,))."""
        P = np.atleast_2d(np.asarray(pts_cam, dtype=np.float64))
        X, Y, Z = P[:, 0], P[:, 1], P[:, 2]
        rho = np.hypot(X, Y)
        theta = np.arctan2(rho, Z)                      # vale também para Z ≤ 0
        theta_d = self._distort_theta(theta)
        scale = np.where(rho > 1e-12, theta_d / np.maximum(rho, 1e-12), 0.0)
        u = self.p.fx * scale * X + self.p.cx
        v = self.p.fy * scale * Y + self.p.cy
        inside = (u >= 0) & (u < self.p.width) & (v >= 0) & (v < self.p.height)
        return np.stack([u, v], axis=1), (Z > 1e-6) & inside

    def pixel_rays(self) -> np.ndarray:
        """Raio unitário de cada pixel, no referencial da câmera. (H, W, 3)."""
        H, W = self.p.height, self.p.width
        u, v = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
        a = (u - self.p.cx) / self.p.fx
        b = (v - self.p.cy) / self.p.fy
        theta_d = np.hypot(a, b)
        theta = self._undistort_theta(theta_d)
        s = np.where(theta_d > 1e-12, np.sin(theta) / np.maximum(theta_d, 1e-12), 1.0)
        return np.stack([a * s, b * s, np.cos(theta)], axis=-1)

    # ---------------- referenciais ----------------

    def car_to_cam(self, pts_car: np.ndarray) -> np.ndarray:
        P = np.atleast_2d(np.asarray(pts_car, dtype=np.float64))
        return (P - self.t_car_cam) @ self.R_car_cam   # = R^T (p − t)

    def world_to_car(self, pts_world: np.ndarray, pose: tuple[float, float, float]) -> np.ndarray:
        x, y, yaw = pose
        P = np.atleast_2d(np.asarray(pts_world, dtype=np.float64))
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = P[:, 0] - x, P[:, 1] - y
        out = np.empty_like(P)
        out[:, 0] = c * dx + s * dy
        out[:, 1] = -s * dx + c * dy
        out[:, 2] = P[:, 2]
        return out

    def project_world(self, pts_world: np.ndarray, pose: tuple[float, float, float]):
        return self.project_cam(self.car_to_cam(self.world_to_car(pts_world, pose)))

    # ---------------- chão ----------------

    def ground_lookup(self, max_range: float = 15.0) -> GroundLookup:
        """Onde cada pixel toca o chão. Calculado uma vez por câmera.

        Como a altura e o pitch da câmera em relação ao chão são FIXOS, o
        ponto de toque no referencial do carro não depende da pose. Por
        quadro, o renderizador só gira e translada esta grade.
        """
        rays = self.pixel_rays().reshape(-1, 3) @ self.R_car_cam.T
        cz = self.t_car_cam[2]
        dz = rays[:, 2]
        hits = dz < -1e-6
        t = np.where(hits, -cz / np.where(hits, dz, -1.0), 0.0)
        fwd = self.t_car_cam[0] + t * rays[:, 0]
        left = self.t_car_cam[1] + t * rays[:, 1]
        valid = hits & (np.hypot(fwd, left) < max_range)
        H, W = self.p.height, self.p.width
        return GroundLookup(
            forward=fwd.reshape(H, W).astype(np.float32),
            left=left.reshape(H, W).astype(np.float32),
            valid=valid.reshape(H, W),
        )

    # ---------------- análise de campo de visão ----------------

    def vertical_half_fov(self) -> float:
        """Meio-FOV vertical no centro da imagem (rad)."""
        theta_d = (self.p.height / 2) / self.p.fy
        return float(self._undistort_theta(np.array([theta_d]))[0])

    def top_edge_elevation(self) -> float:
        """Elevação (rad) da borda superior, no centro, acima do horizonte."""
        return self.vertical_half_fov() - self.p.pitch
