"""Esquema do dataset: um registro por frame gravado.

Formato em disco (ver ``docs/05-formato-dataset.md``):

    data/sessions/<sessao>/
        session.json      metadados da sessão (uma vez)
        records.jsonl     um JSON por linha, um por frame
        images/000001.jpg

Escolhas de formato, e por quê:

* **JSONL** em vez de CSV: campos aninhados (distâncias, flags) cabem sem
  gambiarra, e o arquivo continua sendo *append-only* — se a energia cair no
  meio da gravação, perdemos a última linha, não o arquivo inteiro.
* **JPEG solto** em vez de vídeo: acesso aleatório barato no treino, e um
  frame corrompido não invalida a sessão. Custa ~15% mais disco que H.264 e
  vale a pena.

Módulo de biblioteca padrão apenas — o loader de treino (que usa torch)
importa daqui, mas o inverso nunca acontece.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Iterator

SCHEMA_VERSION = 1
"""Suba ao mudar campos de forma incompatível e registre em docs/05."""

RECORDS_FILENAME = "records.jsonl"
SESSION_FILENAME = "session.json"
IMAGES_DIRNAME = "images"


class SchemaError(ValueError):
    """Registro fora do esquema."""


@dataclass
class Record:
    """Uma amostra: imagem + estado do veículo no instante da captura.

    Atenção aos rótulos (ver ADR 0002):

    * :attr:`steer` / :attr:`throttle` — o que o ESP32 **aplicou**. É com estes
      que se treina.
    * :attr:`cmd_steer` / :attr:`cmd_throttle` — o que o Jetson **pediu**.
      Só para diagnóstico. **Nunca treine com estes.**
    """

    # --- identificação ---
    index: int
    """Índice sequencial dentro da sessão, começando em 0."""

    image: str
    """Caminho relativo à raiz da sessão, ex.: ``images/000042.jpg``."""

    t_host_ns: int
    """Relógio monotônico do Jetson no instante da captura do frame."""

    # --- rótulos (valores aplicados) ---
    steer: float
    """[-1, +1]. Negativo = esquerda, positivo = direita."""

    throttle: float
    """[-1, +1]. Negativo = ré."""

    # --- contexto do veículo ---
    t_mcu_ms: int = 0
    """``millis()`` do ESP32 na telemetria usada como rótulo."""

    telemetry_age_ms: float = 0.0
    """Idade da telemetria no momento do frame. Alto = enlace ruim."""

    source: int = 1
    """Origem do comando aplicado: 0=failsafe, 1=serial, 2=rádio RC."""

    flags: int = 0
    """Bitfield de ``$TLM`` (ver ``protocol.Flags``)."""

    distances_mm: list[int] = field(default_factory=list)
    """Distâncias dos sensores, em mm. 0 = leitura inválida."""

    vbat: float = 0.0
    """Tensão da bateria de tração. Útil para explicar queda de desempenho."""

    # --- diagnóstico (não usar como rótulo) ---
    cmd_steer: float | None = None
    cmd_throttle: float | None = None

    def to_json(self) -> str:
        """Serializa em uma linha, sem espaços supérfluos."""
        data = {k: v for k, v in asdict(self).items() if v is not None}
        return json.dumps(data, separators=(",", ":"), ensure_ascii=False)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Record:
        missing = REQUIRED_FIELDS - data.keys()
        if missing:
            raise SchemaError(f"campos obrigatórios ausentes: {sorted(missing)}")
        known = {k: v for k, v in data.items() if k in _FIELD_NAMES}
        record = cls(**known)  # type: ignore[arg-type]
        record.validate()
        return record

    def validate(self) -> None:
        """Confere invariantes que o treino assume como verdadeiras."""
        if self.index < 0:
            raise SchemaError(f"index negativo: {self.index}")
        if not self.image:
            raise SchemaError("campo 'image' vazio")
        if not -1.0 <= self.steer <= 1.0:
            raise SchemaError(f"steer fora de [-1, 1]: {self.steer}")
        if not -1.0 <= self.throttle <= 1.0:
            raise SchemaError(f"throttle fora de [-1, 1]: {self.throttle}")
        if any(d < 0 for d in self.distances_mm):
            raise SchemaError(f"distância negativa: {self.distances_mm}")

    @property
    def usable_for_training(self) -> bool:
        """False para failsafe (0x02), corte por obstáculo (0x08) ou
        bateria crítica (0x40) — ver ``protocol.UNSAFE_FOR_TRAINING``."""
        return not (self.flags & 0x4A)


_FIELD_NAMES = frozenset(Record.__dataclass_fields__)
REQUIRED_FIELDS = frozenset({"index", "image", "t_host_ns", "steer", "throttle"})


@dataclass
class SessionMeta:
    """Metadados da sessão. Gravado uma vez, no início.

    O que está aqui é o que permite, dois meses depois, saber se uma sessão
    ainda vale: mexeu na câmera ou na calibração da direção, e os dados
    anteriores deixam de ser comparáveis.
    """

    session_id: str
    schema_version: int = SCHEMA_VERSION
    started_at: str = ""
    """ISO-8601 local, ex.: ``2026-08-12T14:33:07-03:00``."""

    ended_at: str = ""
    duration_s: float = 0.0
    frame_count: int = 0

    # --- contexto operacional (perguntado ao operador) ---
    track: str = ""
    driver: str = ""
    lighting: str = ""
    direction: str = ""
    notes: str = ""

    # --- rastreabilidade ---
    vehicle: str = ""
    category: str = ""
    git_commit: str = ""
    hostname: str = ""
    robocar_version: str = ""

    # --- snapshot da configuração usada ---
    camera: dict[str, Any] = field(default_factory=dict)
    capture: dict[str, Any] = field(default_factory=dict)
    vehicle_config: dict[str, Any] = field(default_factory=dict)

    # --- qualidade da coleta ---
    dropped_frames: int = 0
    """Frames descartados por fila cheia (disco lento)."""

    skipped_stopped: int = 0
    """Frames descartados por carro parado."""

    skipped_stale: int = 0
    """Frames descartados por telemetria velha demais."""

    skipped_unsafe: int = 0
    """Frames descartados por failsafe / obstáculo / bateria crítica."""

    discarded: bool = False
    """Marcado por ``robocar dataset tag --discard``."""

    discard_reason: str = ""

    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=2, ensure_ascii=False)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> SessionMeta:
        known = {k: v for k, v in data.items() if k in SessionMeta.__dataclass_fields__}
        return cls(**known)  # type: ignore[arg-type]

    @property
    def yield_ratio(self) -> float:
        """Fração de frames aproveitados. Abaixo de ~0.5, investigue o motivo."""
        total = (
            self.frame_count
            + self.dropped_frames
            + self.skipped_stopped
            + self.skipped_stale
            + self.skipped_unsafe
        )
        return self.frame_count / total if total else 0.0


# ---------------------------------------------------------------------------
# Leitura e escrita
# ---------------------------------------------------------------------------


def write_session_meta(session_dir: Path, meta: SessionMeta) -> None:
    (Path(session_dir) / SESSION_FILENAME).write_text(meta.to_json(), encoding="utf-8")


def read_session_meta(session_dir: Path) -> SessionMeta:
    path = Path(session_dir) / SESSION_FILENAME
    return SessionMeta.from_dict(json.loads(path.read_text(encoding="utf-8")))


def iter_records(session_dir: Path, *, strict: bool = False) -> Iterator[Record]:
    """Percorre ``records.jsonl``.

    Args:
        strict: se True, levanta na primeira linha inválida. Se False (padrão),
            pula linhas quebradas — o caso comum é a última linha ter sido
            truncada por queda de energia, e perder a sessão inteira por causa
            disso seria absurdo.
    """
    path = Path(session_dir) / RECORDS_FILENAME
    if not path.exists():
        raise FileNotFoundError(f"sessão sem {RECORDS_FILENAME}: {session_dir}")

    with path.open("r", encoding="utf-8") as handle:
        for lineno, line in enumerate(handle, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                yield Record.from_dict(json.loads(line))
            except (json.JSONDecodeError, SchemaError) as exc:
                if strict:
                    raise SchemaError(f"{path}:{lineno}: {exc}") from exc
                continue


def read_records(session_dir: Path, *, strict: bool = False) -> list[Record]:
    return list(iter_records(session_dir, strict=strict))


def find_sessions(root: Path) -> list[Path]:
    """Lista diretórios de sessão sob ``root``, ordenados por nome (= por data)."""
    root = Path(root)
    if not root.exists():
        return []
    return sorted(
        p for p in root.iterdir() if p.is_dir() and (p / SESSION_FILENAME).exists()
    )


__all__ = [
    "IMAGES_DIRNAME",
    "RECORDS_FILENAME",
    "REQUIRED_FIELDS",
    "SCHEMA_VERSION",
    "SESSION_FILENAME",
    "Record",
    "SchemaError",
    "SessionMeta",
    "find_sessions",
    "iter_records",
    "read_records",
    "read_session_meta",
    "write_session_meta",
]
