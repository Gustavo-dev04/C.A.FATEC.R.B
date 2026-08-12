"""Sensores do veículo. A câmera vive aqui; as distâncias chegam via ESP32."""

from .camera import Camera, CameraError, open_camera

__all__ = ["Camera", "CameraError", "open_camera"]
