"""Carrega a configuração do carro e da câmera — a fonte única de verdade.

O Vista-Sim lê exatamente os mesmos arquivos que o Vista real
(CLAUDE.md §5A). Calibrou a câmera de verdade → o simulador muda junto.
**Nunca** duplique aqui um número que já está em ``config/``.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


def repo_root() -> Path:
    if env := os.environ.get("VISTA_ROOT"):
        return Path(env)
    here = Path(__file__).resolve()
    for p in (here, *here.parents):
        if (p / "config" / "car.yaml").exists():
            return p
    raise FileNotFoundError("raiz do repositório não encontrada (procurei config/car.yaml)")


def _load(path: Path) -> dict[str, Any]:
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


@dataclass(frozen=True)
class CarParams:
    track_width: float
    wheel_radius: float
    length: float
    width: float
    front_overhang: float      # frente do carro a partir de base_link
    rear_overhang: float
    caster_x: float
    rpm_no_load: float
    time_constant: float
    max_wheel_accel: float
    counts_per_rev: int
    v_max: float
    omega_max: float
    accel_max: float
    alpha_max: float

    @property
    def wheel_speed_max(self) -> float:
        """Velocidade máxima da superfície da roda (m/s)."""
        return self.rpm_no_load / 60.0 * 2.0 * math.pi * self.wheel_radius

    @classmethod
    def from_yaml(cls, data: dict[str, Any]) -> CarParams:
        ch, dr, lim = data["chassis"], data["drive"], data["limits"]
        # O desenho cota x da traseira; o eixo fica a 60 mm dela (folha M3).
        rear = 0.060
        return cls(
            track_width=dr["track_width"],
            wheel_radius=dr["wheel_radius"],
            length=ch["overall_length"],
            width=ch["overall_width"],
            front_overhang=ch["overall_length"] - rear,
            rear_overhang=rear,
            caster_x=dr["caster"]["x"],
            rpm_no_load=dr["motor"]["rpm_no_load"],
            time_constant=dr["motor"]["time_constant"],
            max_wheel_accel=dr["motor"]["max_wheel_accel"],
            counts_per_rev=int(dr["encoder"]["counts_per_rev"]),
            v_max=lim["v_max"],
            omega_max=lim["omega_max"],
            accel_max=lim["accel_max"],
            alpha_max=lim["alpha_max"],
        )


@dataclass(frozen=True)
class CameraParams:
    # intrínsecos na resolução de referência
    fx: float
    fy: float
    cx: float
    cy: float
    dist: tuple[float, float, float, float]
    ref_width: int
    ref_height: int
    calibrated: bool
    # pose em base_link
    x: float
    y: float
    z: float
    pitch: float               # rad, positivo = para baixo
    roll: float
    yaw: float
    # resolução de processamento
    width: int
    height: int

    def scaled(self, width: int, height: int) -> CameraParams:
        """Os mesmos intrínsecos reescalados para outra resolução (mesmo FOV)."""
        sx, sy = width / self.ref_width, height / self.ref_height
        return CameraParams(
            fx=self.fx * sx, fy=self.fy * sy, cx=self.cx * sx, cy=self.cy * sy,
            dist=self.dist, ref_width=width, ref_height=height, calibrated=self.calibrated,
            x=self.x, y=self.y, z=self.z, pitch=self.pitch, roll=self.roll, yaw=self.yaw,
            width=width, height=height,
        )

    def with_pitch(self, pitch_deg: float) -> CameraParams:
        d = self.__dict__.copy()
        d["pitch"] = math.radians(pitch_deg)
        return CameraParams(**d)

    @classmethod
    def from_yaml(cls, intr: dict[str, Any], extr: dict[str, Any]) -> CameraParams:
        proc = extr.get("processing", {})
        base = cls(
            fx=float(intr["fx"]), fy=float(intr["fy"]), cx=float(intr["cx"]), cy=float(intr["cy"]),
            dist=tuple(float(v) for v in intr.get("dist", [0, 0, 0, 0]))[:4],  # type: ignore[arg-type]
            ref_width=int(intr["width"]), ref_height=int(intr["height"]),
            calibrated=bool(intr.get("calibrated", False)),
            x=float(extr["x"]), y=float(extr["y"]), z=float(extr["z"]),
            pitch=math.radians(float(extr["pitch_deg"])),
            roll=math.radians(float(extr.get("roll_deg", 0.0))),
            yaw=math.radians(float(extr.get("yaw_deg", 0.0))),
            width=int(intr["width"]), height=int(intr["height"]),
        )
        return base.scaled(int(proc.get("width", base.width)), int(proc.get("height", base.height)))


def load_car(root: Path | None = None) -> CarParams:
    return CarParams.from_yaml(_load((root or repo_root()) / "config" / "car.yaml"))


def load_camera(root: Path | None = None) -> CameraParams:
    r = root or repo_root()
    return CameraParams.from_yaml(
        _load(r / "config" / "camera" / "intrinsics.yaml"),
        _load(r / "config" / "camera" / "extrinsics.yaml"),
    )
