"""Captura de imagem: CSI (Jetson), webcam USB e replay de sessão gravada.

Todas as implementações devolvem ``(frame_bgr, t_host_ns)``, com o carimbo de
tempo tirado do relógio **monotônico** — o relógio de parede pode saltar (NTP,
fuso) e estragar o alinhamento com a telemetria.

``cv2`` é importado de forma preguiçosa: o resto do pacote precisa continuar
importável em máquinas sem OpenCV.
"""

from __future__ import annotations

import logging
import time
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any

log = logging.getLogger(__name__)


class CameraError(RuntimeError):
    """Falha ao abrir ou ler a câmera."""


def _require_cv2() -> Any:
    try:
        import cv2  # type: ignore[import-not-found]
    except ImportError as exc:  # pragma: no cover - ambiente sem OpenCV
        raise CameraError(
            "OpenCV não encontrado. No Jetson ele vem com o JetPack; "
            "em outras máquinas: pip install opencv-python"
        ) from exc
    return cv2


class Camera(ABC):
    """Interface comum das fontes de imagem."""

    width: int
    height: int
    fps: int

    @abstractmethod
    def read(self) -> tuple[Any, int]:
        """Devolve ``(frame_bgr, t_host_ns)``. Levanta :class:`CameraError`."""

    @abstractmethod
    def release(self) -> None: ...

    def __enter__(self) -> Camera:
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.release()


def build_gstreamer_pipeline(
    sensor_id: int = 0,
    width: int = 1280,
    height: int = 720,
    fps: int = 30,
    flip_method: int = 0,
    *,
    auto_exposure: bool = False,
    exposure_time_ns: int = 8_000_000,
    gain_range: str = "1 4",
    awb_mode: int = 1,
    wb_gains: str = "1.4 1.0 1.0 1.6",
    saturation: float = 1.0,
) -> str:
    """Monta o pipeline ``nvarguscamerasrc`` da câmera CSI.

    Fixar exposição e balanço de branco não é preciosismo: com auto-exposure
    ligado, o ISP reage à faixa branca da pista e muda o brilho da cena inteira.
    A rede aprende esse degrau de brilho como se fosse informação sobre a
    trajetória e passa a errar quando a iluminação do ginásio mudar.
    """
    props = [
        f"nvarguscamerasrc sensor-id={sensor_id}",
        f"saturation={saturation}",
        f"wbmode={awb_mode}",
    ]
    if not auto_exposure:
        props.append(f"exposuretimerange='{exposure_time_ns} {exposure_time_ns}'")
        props.append(f"gainrange='{gain_range}'")
        props.append("aelock=true")
        if awb_mode == 1:
            props.append(f"awblock=true wbgains='{wb_gains}'")

    return (
        " ".join(props)
        + f" ! video/x-raw(memory:NVMM),width={width},height={height},"
        f"framerate={fps}/1,format=NV12"
        f" ! nvvidconv flip-method={flip_method}"
        f" ! video/x-raw,width={width},height={height},format=BGRx"
        " ! videoconvert ! video/x-raw,format=BGR"
        " ! appsink drop=true max-buffers=2 sync=false"
    )


class CsiCamera(Camera):
    """Câmera CSI do Jetson (IMX219/IMX477) via GStreamer."""

    def __init__(
        self,
        sensor_id: int = 0,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        flip_method: int = 0,
        isp: dict[str, Any] | None = None,
    ) -> None:
        cv2 = _require_cv2()
        self.width, self.height, self.fps = width, height, fps
        pipeline = build_gstreamer_pipeline(
            sensor_id, width, height, fps, flip_method, **(isp or {})
        )
        log.debug("pipeline gstreamer: %s", pipeline)
        self._cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self._cap.isOpened():
            raise CameraError(
                "não foi possível abrir a câmera CSI. Verifique com:\n"
                "  ls /dev/video*\n"
                "  gst-inspect-1.0 nvarguscamerasrc\n"
                "Se a câmera estiver em uso por outro processo, rode: "
                "sudo systemctl restart nvargus-daemon"
            )

    def read(self) -> tuple[Any, int]:
        ok, frame = self._cap.read()
        stamp = time.monotonic_ns()
        if not ok or frame is None:
            raise CameraError("falha ao ler frame da câmera CSI")
        return frame, stamp

    def release(self) -> None:
        if getattr(self, "_cap", None) is not None:
            self._cap.release()


class V4l2Camera(Camera):
    """Webcam USB comum. Usada em bancada, sem o Jetson."""

    def __init__(
        self,
        device: str = "/dev/video0",
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
    ) -> None:
        cv2 = _require_cv2()
        self.width, self.height, self.fps = width, height, fps
        self._cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            raise CameraError(f"não foi possível abrir {device}")
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS, fps)
        # Buffer de 1: sempre o frame mais novo. Um buffer maior devolve
        # imagens velhas e o carro passa a dirigir pelo passado.
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if (actual_w, actual_h) != (width, height):
            log.warning(
                "câmera entregou %dx%d em vez de %dx%d", actual_w, actual_h, width, height
            )
            self.width, self.height = actual_w, actual_h

    def read(self) -> tuple[Any, int]:
        ok, frame = self._cap.read()
        stamp = time.monotonic_ns()
        if not ok or frame is None:
            raise CameraError("falha ao ler frame da webcam")
        return frame, stamp

    def release(self) -> None:
        if getattr(self, "_cap", None) is not None:
            self._cap.release()


class FileCamera(Camera):
    """Reproduz as imagens de uma sessão gravada.

    Permite desenvolver e testar todo o pipeline sem carro na mesa — e
    reproduzir exatamente o quadro em que algo deu errado.
    """

    def __init__(
        self,
        session_dir: Path,
        fps: int = 20,
        loop: bool = False,
        realtime: bool = True,
    ) -> None:
        cv2 = _require_cv2()
        self._cv2 = cv2
        self.session_dir = Path(session_dir)
        images_dir = self.session_dir / "images"
        if not images_dir.exists():
            raise CameraError(f"sessão sem diretório de imagens: {images_dir}")

        self._paths = sorted(images_dir.glob("*.jpg"))
        if not self._paths:
            raise CameraError(f"nenhuma imagem em {images_dir}")

        self.fps = fps
        self.loop = loop
        self.realtime = realtime
        self._index = 0
        self._next_ns = time.monotonic_ns()

        first = cv2.imread(str(self._paths[0]))
        if first is None:
            raise CameraError(f"não foi possível ler {self._paths[0]}")
        self.height, self.width = first.shape[:2]

    def __len__(self) -> int:
        return len(self._paths)

    def read(self) -> tuple[Any, int]:
        if self._index >= len(self._paths):
            if not self.loop:
                raise CameraError("fim da sessão")
            self._index = 0

        if self.realtime:
            now = time.monotonic_ns()
            if now < self._next_ns:
                time.sleep((self._next_ns - now) / 1e9)
            self._next_ns += int(1e9 / self.fps)

        path = self._paths[self._index]
        self._index += 1
        frame = self._cv2.imread(str(path))
        if frame is None:
            raise CameraError(f"não foi possível ler {path}")
        return frame, time.monotonic_ns()

    def release(self) -> None:
        return None


def open_camera(config: Any, *, override_backend: str | None = None) -> Camera:
    """Instancia a câmera a partir de ``config/camera.yaml``.

    Args:
        config: objeto :class:`robocar.config.Config`.
        override_backend: força ``csi``/``v4l2``/``file``, ignorando o YAML.
    """
    backend = override_backend or config.get_str("camera.backend", "csi")
    width = config.get_int("camera.capture.width", 1280)
    height = config.get_int("camera.capture.height", 720)
    fps = config.get_int("camera.capture.fps", 30)

    if backend == "csi":
        return CsiCamera(
            sensor_id=config.get_int("camera.sensor_id", 0),
            width=width,
            height=height,
            fps=fps,
            flip_method=config.get_int("camera.capture.flip_method", 0),
            isp=config.get("camera.isp", {}),
        )
    if backend == "v4l2":
        return V4l2Camera(
            device=config.get_str("camera.device", "/dev/video0"),
            width=width,
            height=height,
            fps=fps,
        )
    if backend == "file":
        return FileCamera(Path(config.get_str("camera.replay_session")), fps=fps)
    raise CameraError(f"backend de câmera desconhecido: {backend!r}")


def sharpness(frame: Any, roi: dict[str, float] | None = None) -> float:
    """Mede a nitidez do quadro pela variância do laplaciano.

    Quanto maior, mais nítido. É a métrica que permite ajustar a rosca de foco
    da lente por número em vez de "achismo": gire devagar até o valor parar de
    subir, depois recue até o pico.

    O recorte é redimensionado para uma largura fixa antes da medição, de modo
    que o número seja comparável entre resoluções de captura diferentes —
    caso contrário, mudar de 1280×720 para 1640×1232 alteraria a escala e a
    referência anotada em ``config/camera.yaml`` perderia sentido.
    """
    cv2 = _require_cv2()
    import numpy as np

    region = crop_roi(frame, roi) if roi else frame
    if region.size == 0:
        return 0.0

    height, width = region.shape[:2]
    if width != _SHARPNESS_WIDTH:
        scale = _SHARPNESS_WIDTH / width
        region = cv2.resize(
            region,
            (_SHARPNESS_WIDTH, max(1, int(height * scale))),
            interpolation=cv2.INTER_AREA,
        )

    gray = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY)
    return float(np.var(cv2.Laplacian(gray, cv2.CV_64F)))


_SHARPNESS_WIDTH = 640
"""Largura de referência da medição de nitidez. Não mude sem refazer as
referências anotadas em ``config/camera.yaml``."""


def crop_roi(frame: Any, roi: dict[str, float]) -> Any:
    """Recorta uma região definida em frações da imagem (0.0–1.0).

    Aceita as chaves ``top``, ``bottom``, ``left`` e ``right``; as ausentes
    viram a borda correspondente.
    """
    height, width = frame.shape[:2]
    top = int(height * float(roi.get("top", 0.0)))
    bottom = int(height * float(roi.get("bottom", 1.0)))
    left = int(width * float(roi.get("left", 0.0)))
    right = int(width * float(roi.get("right", 1.0)))
    return frame[max(top, 0) : min(bottom, height), max(left, 0) : min(right, width)]


__all__ = [
    "Camera",
    "CameraError",
    "CsiCamera",
    "FileCamera",
    "V4l2Camera",
    "build_gstreamer_pipeline",
    "crop_roi",
    "open_camera",
    "sharpness",
]
