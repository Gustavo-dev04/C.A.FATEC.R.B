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
    height: float              # topo da pilha (para a sombra do próprio carro)
    front_overhang: float      # frente do carro a partir de base_link
    rear_overhang: float
    caster_x: float
    # motor (por roda)
    rpm_no_load: float
    motor_tau: float
    deadband_duty: float
    # encoder
    counts_per_rev: int
    speed_window_s: float
    # PID de velocidade por roda (ESP32)
    pid_hz: float
    kp: float
    ki: float
    kd: float
    feedforward: bool
    target_limit: float
    # tolerâncias
    radius_mismatch: float
    wheel_slip: float
    # limites
    v_max: float
    omega_max: float

    @property
    def center_x(self) -> float:
        """Centro do carro, à frente do eixo (m). É o ponto que segue a faixa:
        com o eixo na faixa a frente (34 cm adiante) balança para fora nas
        curvas; com a câmera, a traseira corta a quina. O centro passa em
        todas as conversões da malha (varredura em tests/test_cenario.py)."""
        return (self.front_overhang - self.rear_overhang) / 2.0

    @property
    def omega_wheel_max(self) -> float:
        """Rotação livre da roda (rad/s)."""
        return self.rpm_no_load / 60.0 * 2.0 * math.pi

    @property
    def wheel_speed_max(self) -> float:
        """Velocidade máxima da superfície da roda (m/s)."""
        return self.omega_wheel_max * self.wheel_radius

    @classmethod
    def from_yaml(cls, data: dict[str, Any]) -> CarParams:
        ch, dr, lim = data["chassis"], data["drive"], data["limits"]
        mo, enc, pid, tol = dr["motor"], dr["encoder"], dr["pid"], dr.get("tolerances", {})
        # O desenho cota x da traseira; o eixo fica a 60 mm dela (folha M3).
        rear = 0.060
        return cls(
            track_width=dr["track_width"],
            wheel_radius=dr["wheel_radius"],
            length=ch["overall_length"],
            width=ch["overall_width"],
            # P02 (acrílico) a 135,5 mm + Jetson/suportes: ~0,17 m (estimativa)
            height=float(ch.get("stack_height", 0.17)),
            front_overhang=ch["overall_length"] - rear,
            rear_overhang=rear,
            caster_x=dr["caster"]["x"],
            rpm_no_load=float(mo["rpm_no_load"]),
            motor_tau=float(mo["mechanical_time_constant"]),
            deadband_duty=float(mo["deadband_duty"]),
            counts_per_rev=int(enc["counts_per_rev"]),
            speed_window_s=float(enc.get("speed_window_s", 0.02)),
            pid_hz=float(pid["hz"]),
            kp=float(pid["kp"]),
            ki=float(pid["ki"]),
            kd=float(pid.get("kd", 0.0)),
            feedforward=bool(pid.get("feedforward", True)),
            target_limit=float(pid.get("target_limit", 0.92)),
            radius_mismatch=float(tol.get("wheel_radius_mismatch", 0.0)),
            wheel_slip=float(tol.get("wheel_slip", 0.0)),
            v_max=lim["v_max"],
            omega_max=lim["omega_max"],
        )


@dataclass(frozen=True)
class ImuParams:
    """MPU6050: números do datasheet + o que só se mede no carro."""

    model: str
    x: float                     # posição em base_link (m)
    y: float
    gyro_range: float            # rad/s
    gyro_noise_density: float    # rad/s/√Hz
    gyro_initial_bias: float     # rad/s (tolerância de fábrica)
    gyro_scale_tolerance: float  # fração
    gyro_bias_drift: float       # rad/s em ~100 s
    accel_range: float           # m/s²
    accel_noise_density: float   # m/s²/√Hz
    accel_initial_bias: tuple[float, float, float]   # m/s²
    dlpf_hz: float

    @classmethod
    def from_yaml(cls, d: dict[str, Any]) -> ImuParams:
        dps = math.pi / 180.0
        g = 9.80665
        return cls(
            model=str(d["model"]), x=float(d["x"]), y=float(d.get("y", 0.0)),
            gyro_range=float(d["gyro_range_dps"]) * dps,
            gyro_noise_density=float(d["gyro_noise_density_dps_rthz"]) * dps,
            gyro_initial_bias=float(d["gyro_initial_bias_dps"]) * dps,
            gyro_scale_tolerance=float(d.get("gyro_scale_tolerance", 0.0)),
            gyro_bias_drift=float(d.get("gyro_bias_drift_dps", 0.0)) * dps,
            accel_range=float(d["accel_range_g"]) * g,
            accel_noise_density=float(d["accel_noise_density_ug_rthz"]) * 1e-6 * g,
            accel_initial_bias=tuple(float(v) * 1e-3 * g for v in d.get("accel_initial_bias_mg", [0, 0, 0])),  # type: ignore[arg-type]
            dlpf_hz=float(d.get("dlpf_hz", 98.0)),
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


def load_imu(root: Path | None = None) -> ImuParams:
    return ImuParams.from_yaml(_load((root or repo_root()) / "config" / "car.yaml")["imu"])


def load_camera(root: Path | None = None) -> CameraParams:
    r = root or repo_root()
    return CameraParams.from_yaml(
        _load(r / "config" / "camera" / "intrinsics.yaml"),
        _load(r / "config" / "camera" / "extrinsics.yaml"),
    )
