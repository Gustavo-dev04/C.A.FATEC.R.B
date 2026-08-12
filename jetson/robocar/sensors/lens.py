"""Calibração da lente e câmera virtual — otimização por software dos 120°.

A lente de 120° é ótima para manutenção de faixa (vê a borda interna da rua na
curva fechada) e ruim para placas (poucos pixels, e distorção de barril
justamente na borda direita, onde as placas ficam).

Este módulo resolve o segundo problema **por código**, sem trocar a lente:

    ┌──────────────────────────────────────┐
    │  quadro bruto 1640×1232, 120°        │
    │                          ╭───────╮   │   a placa aparece deformada
    │                          │ placa │   │   pela distorção de barril
    │        rua               ╰───────╯   │
    │                                      │
    └──────────────────────────────────────┘
                     │  VirtualCamera(yaw=+30°, fov=50°)
                     ▼
              ┌─────────────┐
              │   ╭─────╮   │   vista retificada, "como se" houvesse uma
              │   │placa│   │   segunda câmera estreita apontada para a
              │   ╰─────╯   │   direita: círculo é círculo, seta é reta
              └─────────────┘

Uma ressalva honesta: **retificar não cria pixels**. A placa não fica mais
nítida — fica geometricamente correta e sempre no mesmo enquadramento. O ganho
é de precisão do classificador (ele para de ter que aprender todas as
deformações possíveis), não de resolução.

A matemática de FOV e intrínsecos é Python puro e testada sem hardware; só a
construção dos mapas de remapeamento precisa de OpenCV.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


class CalibrationError(RuntimeError):
    """Calibração ausente, incompleta ou incoerente."""


# ---------------------------------------------------------------------------
# Matemática pura (testável sem OpenCV)
# ---------------------------------------------------------------------------


def focal_px_from_fov(fov_deg: float, pixels: int) -> float:
    """Distância focal em pixels correspondente a um campo de visão.

    Modelo pinhole: ``f = (pixels/2) / tan(fov/2)``.
    """
    if not 0 < fov_deg < 180:
        raise ValueError(f"fov fora de (0, 180): {fov_deg}")
    return (pixels / 2.0) / math.tan(math.radians(fov_deg) / 2.0)


def fov_deg_from_focal_px(focal_px: float, pixels: int) -> float:
    """Inverso de :func:`focal_px_from_fov`."""
    if focal_px <= 0:
        raise ValueError(f"focal precisa ser positiva: {focal_px}")
    return math.degrees(2.0 * math.atan((pixels / 2.0) / focal_px))


def object_px(size_mm: float, distance_mm: float, focal_px: float) -> float:
    """Tamanho aparente, em pixels, de um objeto de ``size_mm`` a ``distance_mm``.

    É a conta que decide se o detector de placas é viável: uma placa de 150 mm
    precisa de ~32 px. Ver docs/02-hardware.md.
    """
    if distance_mm <= 0:
        raise ValueError("distância precisa ser positiva")
    return size_mm * focal_px / distance_mm


def rotation_matrix(yaw_deg: float, pitch_deg: float) -> list[list[float]]:
    """Rotação da câmera virtual, em coordenadas de câmera (x→direita,
    y→baixo, z→frente).

    ``yaw`` positivo aponta para a **direita** (onde ficam as placas);
    ``pitch`` positivo aponta para **cima**.

    Devolve ``R = Ry(yaw) @ Rx(pitch)``, tal que ``R @ [0,0,1]`` é a direção
    para onde a câmera virtual olha, expressa no referencial da câmera real.
    Como ``y`` cresce para baixo, a componente ``y`` dessa direção é
    ``-sin(pitch)``: pitch positivo dá ``y`` negativo, ou seja, para cima.
    """
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)

    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)

    # Ry(yaw) @ Rx(pitch)
    return [
        [cy, sy * sp, sy * cp],
        [0.0, cp, -sp],
        [-sy, cy * sp, cy * cp],
    ]


def view_direction(yaw_deg: float, pitch_deg: float) -> tuple[float, float, float]:
    """Vetor unitário para onde a câmera virtual aponta. Usado nos testes."""
    r = rotation_matrix(yaw_deg, pitch_deg)
    return (r[0][2], r[1][2], r[2][2])


# ---------------------------------------------------------------------------
# Intrínsecos
# ---------------------------------------------------------------------------


@dataclass
class CameraIntrinsics:
    """Parâmetros intrínsecos e de distorção da câmera.

    Gerados por ``robocar calib camera`` e gravados em
    ``config/calib/camera_intrinsics.yaml``.
    """

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    #: Coeficientes de distorção do modelo de Brown-Conrady: k1,k2,p1,p2,k3.
    #: Numa lente de 120°, k1 é bem negativo — é a distorção de barril.
    dist: list[float] = field(default_factory=lambda: [0.0] * 5)
    rms_error_px: float = 0.0
    """Erro de reprojeção da calibração. Acima de ~1,0 px, refaça."""

    calibrated_at: str = ""
    sample_count: int = 0

    def __post_init__(self) -> None:
        if self.fx <= 0 or self.fy <= 0:
            raise CalibrationError(f"focal inválida: fx={self.fx}, fy={self.fy}")
        if self.width <= 0 or self.height <= 0:
            raise CalibrationError("resolução inválida")

    @property
    def hfov_deg(self) -> float:
        return fov_deg_from_focal_px(self.fx, self.width)

    @property
    def vfov_deg(self) -> float:
        return fov_deg_from_focal_px(self.fy, self.height)

    @property
    def dfov_deg(self) -> float:
        """FOV diagonal — é este que os fabricantes costumam anunciar."""
        diagonal = math.hypot(self.width, self.height)
        focal = (self.fx + self.fy) / 2.0
        return fov_deg_from_focal_px(focal, int(diagonal))

    def scale_to(self, width: int, height: int) -> CameraIntrinsics:
        """Reescala os intrínsecos para outra resolução do mesmo sensor.

        Permite calibrar uma vez em 1640×1232 e usar em 3280×2464 sem
        recalibrar — desde que o modo tenha o **mesmo campo de visão**. Não
        vale para os modos recortados do IMX219 (1920×1080, 1280×720), que
        mudam o FOV e exigem calibração própria.
        """
        sx = width / self.width
        sy = height / self.height
        return CameraIntrinsics(
            fx=self.fx * sx,
            fy=self.fy * sy,
            cx=self.cx * sx,
            cy=self.cy * sy,
            width=width,
            height=height,
            dist=list(self.dist),
            rms_error_px=self.rms_error_px,
            calibrated_at=self.calibrated_at,
            sample_count=self.sample_count,
        )

    def matrix(self) -> list[list[float]]:
        """Matriz K 3×3."""
        return [
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0],
        ]

    def to_dict(self) -> dict[str, Any]:
        return {
            "fx": round(self.fx, 4),
            "fy": round(self.fy, 4),
            "cx": round(self.cx, 4),
            "cy": round(self.cy, 4),
            "width": self.width,
            "height": self.height,
            "dist": [round(d, 8) for d in self.dist],
            "rms_error_px": round(self.rms_error_px, 4),
            "calibrated_at": self.calibrated_at,
            "sample_count": self.sample_count,
            "_hfov_deg": round(self.hfov_deg, 2),
            "_vfov_deg": round(self.vfov_deg, 2),
            "_dfov_deg": round(self.dfov_deg, 2),
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> CameraIntrinsics:
        known = {
            k: v for k, v in data.items()
            if k in cls.__dataclass_fields__ and not k.startswith("_")
        }
        missing = {"fx", "fy", "cx", "cy", "width", "height"} - known.keys()
        if missing:
            raise CalibrationError(f"campos ausentes na calibração: {sorted(missing)}")
        return cls(**known)  # type: ignore[arg-type]

    def save(self, path: Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        header = (
            "# Calibração intrínseca da câmera — gerado por `robocar calib camera`.\n"
            "# Campos com prefixo '_' são informativos (derivados dos demais).\n"
        )
        path.write_text(
            header + yaml.safe_dump(self.to_dict(), sort_keys=False), encoding="utf-8"
        )

    @classmethod
    def load(cls, path: Path) -> CameraIntrinsics:
        path = Path(path)
        if not path.exists():
            raise CalibrationError(
                f"calibração não encontrada: {path}\n"
                "Rode `robocar calib camera` (precisa de um tabuleiro de xadrez impresso)."
            )
        return cls.from_dict(yaml.safe_load(path.read_text(encoding="utf-8")) or {})


# ---------------------------------------------------------------------------
# Câmera virtual
# ---------------------------------------------------------------------------


@dataclass
class VirtualCameraSpec:
    """Definição de uma câmera virtual recortada do quadro grande angular."""

    name: str = "signs"
    yaw_deg: float = 30.0
    """Positivo = para a direita. As placas ficam sempre à direita (item 3.5.2)."""

    pitch_deg: float = 8.0
    """Positivo = para cima. A placa fica a 475 mm do chão, câmera a 180 mm."""

    hfov_deg: float = 50.0
    width: int = 0
    """0 = calcular automaticamente para não perder resolução."""

    height: int = 0

    def resolve(self, intrinsics: CameraIntrinsics) -> VirtualCameraSpec:
        """Preenche largura/altura automáticas.

        O tamanho automático é o que mantém a mesma densidade de pixels por
        grau da câmera real no centro: ``w = 2·f_src·tan(fov/2)``. Menor que
        isso joga fora detalhe; muito maior só interpola.
        """
        if self.width and self.height:
            return self

        width = int(round(2 * intrinsics.fx * math.tan(math.radians(self.hfov_deg) / 2)))
        # Mantém a proporção do recorte quadrada-ish: placas são circulares e
        # aparecem numa faixa vertical estreita.
        height = self.height or int(round(width * 0.75))
        return VirtualCameraSpec(
            name=self.name,
            yaw_deg=self.yaw_deg,
            pitch_deg=self.pitch_deg,
            hfov_deg=self.hfov_deg,
            width=self.width or width,
            height=height,
        )

    def intrinsics(self) -> CameraIntrinsics:
        """Intrínsecos da câmera virtual (pinhole puro, sem distorção)."""
        if not self.width or not self.height:
            raise CalibrationError("chame resolve() antes de intrinsics()")
        focal = focal_px_from_fov(self.hfov_deg, self.width)
        return CameraIntrinsics(
            fx=focal,
            fy=focal,
            cx=self.width / 2.0,
            cy=self.height / 2.0,
            width=self.width,
            height=self.height,
        )

    def sign_px_at(self, distance_mm: float, sign_mm: float = 150.0) -> float:
        """Tamanho da placa, em px, nesta câmera virtual."""
        return object_px(sign_mm, distance_mm, self.intrinsics().fx)


class VirtualCamera:
    """Vista retificada e reapontada, extraída do quadro grande angular.

    Constrói os mapas de remapeamento **uma vez** e depois só aplica
    ``cv2.remap``, que é barato (poucos ms para uma saída de ~500×375) e roda
    na CPU sem atrapalhar a GPU, que está ocupada com a inferência.
    """

    def __init__(self, intrinsics: CameraIntrinsics, spec: VirtualCameraSpec) -> None:
        self.source = intrinsics
        self.spec = spec.resolve(intrinsics)
        self.target = self.spec.intrinsics()
        self._maps: tuple[Any, Any] | None = None
        self._cv2: Any = None

    def build(self) -> VirtualCamera:
        """Pré-computa os mapas. Chamar uma vez, fora do laço de controle."""
        import cv2  # type: ignore[import-not-found]
        import numpy as np

        self._cv2 = cv2

        k_src = np.array(self.source.matrix(), dtype=np.float64)
        dist = np.array(self.source.dist, dtype=np.float64).reshape(-1, 1)
        k_dst = np.array(self.target.matrix(), dtype=np.float64)

        # initUndistortRectifyMap aplica R^-1 ao raio do destino antes de
        # projetá-lo na origem. Como rotation_matrix() devolve a rotação que
        # leva o eixo óptico virtual ao referencial real, passamos a transposta.
        rotation = np.array(
            rotation_matrix(self.spec.yaw_deg, self.spec.pitch_deg), dtype=np.float64
        ).T

        self._maps = cv2.initUndistortRectifyMap(
            k_src,
            dist,
            rotation,
            k_dst,
            (self.spec.width, self.spec.height),
            cv2.CV_16SC2,  # inteiro de ponto fixo: remap ~2x mais rápido
        )
        return self

    def apply(self, frame: Any) -> Any:
        """Extrai a vista virtual de um quadro completo."""
        if self._maps is None:
            self.build()
        assert self._maps is not None
        return self._cv2.remap(
            frame, self._maps[0], self._maps[1], self._cv2.INTER_LINEAR
        )

    def describe(self) -> str:
        spec = self.spec
        return (
            f"câmera virtual '{spec.name}': {spec.width}x{spec.height} @ "
            f"{spec.hfov_deg:.0f}° | mira yaw={spec.yaw_deg:+.0f}° "
            f"pitch={spec.pitch_deg:+.0f}° | placa de 150 mm: "
            f"{spec.sign_px_at(1000):.0f} px a 1 m, "
            f"{spec.sign_px_at(2000):.0f} px a 2 m, "
            f"{spec.sign_px_at(3000):.0f} px a 3 m"
        )


def virtual_camera_from_config(config: Any, intrinsics: CameraIntrinsics) -> VirtualCamera:
    """Constrói a câmera virtual das placas a partir de ``config/camera.yaml``."""
    spec = VirtualCameraSpec(
        name="signs",
        yaw_deg=config.get_float("camera.virtual_sign_camera.yaw_deg", 30.0),
        pitch_deg=config.get_float("camera.virtual_sign_camera.pitch_deg", 8.0),
        hfov_deg=config.get_float("camera.virtual_sign_camera.hfov_deg", 50.0),
        width=config.get_int("camera.virtual_sign_camera.width", 0),
        height=config.get_int("camera.virtual_sign_camera.height", 0),
    )
    return VirtualCamera(intrinsics, spec)


__all__ = [
    "CalibrationError",
    "CameraIntrinsics",
    "VirtualCamera",
    "VirtualCameraSpec",
    "focal_px_from_fov",
    "fov_deg_from_focal_px",
    "object_px",
    "rotation_matrix",
    "view_direction",
    "virtual_camera_from_config",
]
