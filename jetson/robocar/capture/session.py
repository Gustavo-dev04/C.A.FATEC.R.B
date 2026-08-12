"""Criação de diretórios de sessão e coleta de metadados.

Uma sessão é uma unidade de coleta: "10 minutos rodando na minicidade, sentido
horário, luz natural, piloto Gustavo". É também a unidade de divisão
treino/validação — misturar frames da mesma sessão entre os dois conjuntos
infla a métrica de validação sem que o modelo tenha generalizado nada.
"""

from __future__ import annotations

import contextlib
import getpass
import os
import platform
import socket
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Any

from .schema import IMAGES_DIRNAME, SessionMeta, write_session_meta


def git_commit(root: Path | None = None) -> str:
    """Hash curto do commit atual, com ``-sujo`` se houver alteração local.

    Saber exatamente qual código gravou uma sessão vale muito quando duas
    coletas se comportam de formas diferentes.
    """
    try:
        cwd = str(root) if root else None
        commit = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=cwd,
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
        dirty = subprocess.check_output(
            ["git", "status", "--porcelain"],
            cwd=cwd,
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
        return f"{commit}-sujo" if dirty else commit
    except (subprocess.CalledProcessError, FileNotFoundError, OSError):
        return "desconhecido"


def make_session_id(track: str = "", tag: str = "", now: datetime | None = None) -> str:
    """Nome do diretório: ``AAAAMMDD_HHMMSS_pista_tag``.

    Prefixo temporal garante que ordem alfabética = ordem cronológica, o que
    faz ``ls`` e ``sorted()`` fazerem a coisa certa sem esforço.
    """
    stamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
    parts = [stamp, _slug(track), _slug(tag)]
    return "_".join(part for part in parts if part)


def _slug(text: str) -> str:
    """Reduz texto livre a ``[a-z0-9_-]`` — nome de diretório previsível."""
    text = (text or "").strip().lower()
    keep = []
    for char in text:
        if char.isalnum():
            keep.append(char)
        elif char in " -_":
            keep.append("_")
    return "".join(keep).strip("_")[:40]


def create_session(
    root: Path,
    config: Any,
    *,
    track: str = "",
    driver: str = "",
    lighting: str = "",
    direction: str = "",
    notes: str = "",
    tag: str = "",
    repo_root: Path | None = None,
) -> tuple[Path, SessionMeta]:
    """Cria ``<root>/<session_id>/images/`` e devolve ``(caminho, metadados)``."""
    session_id = make_session_id(track=track, tag=tag)
    session_dir = Path(root) / session_id
    (session_dir / IMAGES_DIRNAME).mkdir(parents=True, exist_ok=False)

    meta = SessionMeta(
        session_id=session_id,
        started_at=datetime.now().astimezone().isoformat(timespec="seconds"),
        track=track,
        driver=driver or _current_user(),
        lighting=lighting,
        direction=direction,
        notes=notes,
        vehicle=config.get_str("vehicle.name", ""),
        category=config.get_str("vehicle.category", ""),
        git_commit=git_commit(repo_root),
        hostname=socket.gethostname(),
        robocar_version=_package_version(),
        camera=config.get("camera", {}),
        capture=config.get("capture", {}),
        vehicle_config={
            "steering": config.get("steering", {}),
            "throttle": config.get("throttle", {}),
            "safety": config.get("safety", {}),
            "distance_sensors": config.get("distance_sensors", {}),
            "vehicle": config.get("vehicle", {}),
        },
    )
    write_session_meta(session_dir, meta)
    return session_dir, meta


def _current_user() -> str:
    try:
        return getpass.getuser()
    except Exception:  # pragma: no cover - ambiente sem usuário definido
        return os.environ.get("USER", "desconhecido")


def _package_version() -> str:
    from .. import __version__

    return __version__


def disk_free_gb(path: Path) -> float:
    """Espaço livre em GB no volume que contém ``path``."""
    path = Path(path)
    while not path.exists() and path != path.parent:
        path = path.parent
    usage = os.statvfs(path)
    return usage.f_bavail * usage.f_frsize / (1024**3)


def estimate_session_gb(rate_hz: float, minutes: float, kb_per_frame: float = 150.0) -> float:
    """Estimativa de disco para uma sessão. Usada nos avisos de espaço."""
    return rate_hz * 60.0 * minutes * kb_per_frame / (1024**2)


def system_info() -> dict[str, str]:
    """Contexto da máquina, útil para depurar diferenças entre bancada e carro."""
    info = {
        "hostname": socket.gethostname(),
        "platform": platform.platform(),
        "python": platform.python_version(),
        "machine": platform.machine(),
    }
    # No Jetson, este arquivo identifica o modelo exato da placa.
    model = Path("/proc/device-tree/model")
    if model.exists():
        with contextlib.suppress(OSError):
            info["board"] = model.read_text(errors="ignore").strip("\x00").strip()
    return info


__all__ = [
    "create_session",
    "disk_free_gb",
    "estimate_session_gb",
    "git_commit",
    "make_session_id",
    "system_info",
]
