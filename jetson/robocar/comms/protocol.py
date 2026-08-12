"""Codec do protocolo serial Jetson <-> ESP32.

Especificação normativa: ``docs/03-protocolo-serial.md``.
A implementação equivalente em C++ vive em ``firmware/esp32/src/protocol.cpp``.

Este módulo usa **apenas a biblioteca padrão** de propósito: ele é a camada
mais baixa do sistema e precisa ser testável sem hardware, sem pyserial e sem
numpy. Não adicione dependências aqui.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass
from typing import Iterable

PROTOCOL_VERSION = 1

FRAME_START = "$"
CHECKSUM_SEP = "*"
FRAME_END = "\n"
MAX_FRAME_LEN = 128
"""Tamanho máximo de um quadro, incluindo ``$``, ``*CS`` e ``\\n``."""

_MAX_BUFFER = MAX_FRAME_LEN * 8
"""Teto do buffer do parser. Acima disso presumimos lixo e descartamos."""

SEQ_MODULO = 0x10000


class ErrorCode(enum.IntEnum):
    """Códigos de ``$ERR`` (ver tabela em docs/03-protocolo-serial.md)."""

    BAD_CHECKSUM = 1
    UNKNOWN_TYPE = 2
    BAD_FIELD_COUNT = 3
    OUT_OF_RANGE = 4
    UNKNOWN_CONFIG_KEY = 5
    FRAME_TOO_LONG = 6


class Mode(enum.IntEnum):
    """Modo de operação declarado pelo Jetson em ``$CMD``."""

    IDLE = 0
    TELEOP = 1
    AUTO = 2


class Source(enum.IntEnum):
    """Origem do comando efetivamente aplicado, reportada em ``$TLM``."""

    FAILSAFE = 0
    SERIAL = 1
    RC = 2


class Flags(enum.IntFlag):
    """Bitfield de estado do ESP32 (campo ``flags`` de ``$TLM``)."""

    NONE = 0x00
    ARMED = 0x01
    FAILSAFE = 0x02
    RC_VALID = 0x04
    OBSTACLE_STOP = 0x08
    OBSTACLE_SLOW = 0x10
    BATT_LOW = 0x20
    BATT_CRITICAL = 0x40
    CFG_PENDING = 0x80


#: Flags que invalidam uma amostra para treinamento: nesses instantes o valor
#: aplicado não reflete a intenção do piloto. Ver ADR 0002.
UNSAFE_FOR_TRAINING = Flags.FAILSAFE | Flags.OBSTACLE_STOP | Flags.BATT_CRITICAL


class ProtocolError(Exception):
    """Falha ao decodificar um quadro."""

    def __init__(self, code: ErrorCode, detail: str = "") -> None:
        super().__init__(f"[{code.name}] {detail}")
        self.code = code
        self.detail = detail


# ---------------------------------------------------------------------------
# Utilidades numéricas
# ---------------------------------------------------------------------------


def clamp(value: float, low: float, high: float) -> float:
    """Limita ``value`` ao intervalo fechado ``[low, high]``."""
    return low if value < low else high if value > high else value


def fmt_norm(value: float) -> str:
    """Formata um valor normalizado ([-1, 1]) com 4 casas decimais."""
    return f"{clamp(float(value), -1.0, 1.0):.4f}"


def checksum(payload: str) -> str:
    """XOR de todos os bytes do payload, em 2 dígitos hex maiúsculos.

    ``payload`` é o conteúdo *entre* ``$`` e ``*``, sem incluir nenhum dos dois.
    """
    cs = 0
    for byte in payload.encode("ascii"):
        cs ^= byte
    return f"{cs:02X}"


# ---------------------------------------------------------------------------
# Mensagens
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Message:
    """Base de todas as mensagens decodificadas."""


@dataclass(frozen=True)
class Command(Message):
    """``$CMD`` — comando de atuação (Jetson -> ESP32)."""

    seq: int
    steer: float
    throttle: float
    mode: Mode = Mode.IDLE

    def encode(self) -> str:
        return encode_frame(
            f"CMD,{self.seq % SEQ_MODULO},{fmt_norm(self.steer)},"
            f"{fmt_norm(self.throttle)},{int(self.mode)}"
        )


@dataclass(frozen=True)
class Telemetry(Message):
    """``$TLM`` — telemetria (ESP32 -> Jetson).

    ``steer`` e ``throttle`` são os valores **efetivamente aplicados** aos
    atuadores, já com rampa, limites e cortes de segurança. São eles, e não o
    comando enviado, que servem de rótulo para o dataset (ver ADR 0002).
    """

    seq: int
    t_ms: int
    steer: float
    throttle: float
    source: Source
    distances_mm: tuple[int, ...]
    vbat: float
    flags: Flags

    @property
    def armed(self) -> bool:
        return bool(self.flags & Flags.ARMED)

    @property
    def failsafe(self) -> bool:
        return bool(self.flags & Flags.FAILSAFE)

    @property
    def usable_for_training(self) -> bool:
        """False quando o valor aplicado não reflete a intenção do piloto."""
        return not (self.flags & UNSAFE_FOR_TRAINING)

    def distance(self, index: int) -> int | None:
        """Distância do sensor ``index`` em mm, ou ``None`` se inválida.

        O firmware reporta ``0`` quando não houve eco ou o alvo está fora de
        alcance — o que é bem diferente de "obstáculo a 0 mm".
        """
        if index < 0 or index >= len(self.distances_mm):
            return None
        value = self.distances_mm[index]
        return None if value == 0 else value

    def encode(self) -> str:
        dist = ",".join(str(int(d)) for d in self.distances_mm)
        return encode_frame(
            f"TLM,{self.seq % SEQ_MODULO},{int(self.t_ms)},{fmt_norm(self.steer)},"
            f"{fmt_norm(self.throttle)},{int(self.source)},{dist},"
            f"{float(self.vbat):.2f},{int(self.flags):02X}"
        )


@dataclass(frozen=True)
class Arm(Message):
    """``$ARM`` — arma ou desarma a tração."""

    armed: bool

    def encode(self) -> str:
        return encode_frame(f"ARM,{1 if self.armed else 0}")


@dataclass(frozen=True)
class Config(Message):
    """``$CFG`` — ajuste de parâmetro em runtime."""

    key: str
    value: str

    def encode(self) -> str:
        return encode_frame(f"CFG,{self.key},{self.value}")


@dataclass(frozen=True)
class Ping(Message):
    """``$PING`` — teste de enlace."""

    seq: int

    def encode(self) -> str:
        return encode_frame(f"PING,{self.seq % SEQ_MODULO}")


@dataclass(frozen=True)
class Pong(Message):
    """``$PONG`` — resposta ao ping, com o relógio do ESP32."""

    seq: int
    t_ms: int

    def encode(self) -> str:
        return encode_frame(f"PONG,{self.seq % SEQ_MODULO},{int(self.t_ms)}")


@dataclass(frozen=True)
class Ack(Message):
    """``$ACK`` — confirmação de comando."""

    kind: str
    detail: str = ""

    def encode(self) -> str:
        return encode_frame(f"ACK,{self.kind},{self.detail}")


@dataclass(frozen=True)
class Err(Message):
    """``$ERR`` — erro reportado pelo ESP32."""

    code: int
    detail: str = ""

    def encode(self) -> str:
        return encode_frame(f"ERR,{int(self.code)},{self.detail}")


@dataclass(frozen=True)
class LogMessage(Message):
    """``$LOG`` — texto livre de depuração vindo do firmware."""

    level: str
    text: str

    def encode(self) -> str:
        safe = self.text.replace(",", " ").replace(CHECKSUM_SEP, " ")
        return encode_frame(f"LOG,{self.level},{safe}")


# ---------------------------------------------------------------------------
# Codificação
# ---------------------------------------------------------------------------


def encode_frame(payload: str) -> str:
    """Envolve ``payload`` com ``$``, checksum e terminador de linha."""
    frame = f"{FRAME_START}{payload}{CHECKSUM_SEP}{checksum(payload)}{FRAME_END}"
    if len(frame) > MAX_FRAME_LEN:
        raise ProtocolError(
            ErrorCode.FRAME_TOO_LONG,
            f"quadro com {len(frame)} bytes (máximo {MAX_FRAME_LEN})",
        )
    return frame


def encode_command(seq: int, steer: float, throttle: float, mode: Mode = Mode.AUTO) -> str:
    """Atalho para ``Command(...).encode()``."""
    return Command(seq=seq, steer=steer, throttle=throttle, mode=mode).encode()


def encode_arm(armed: bool) -> str:
    return Arm(armed=armed).encode()


def encode_config(key: str, value: object) -> str:
    """Codifica ``$CFG``.

    Floats saem com 4 casas; inteiros e booleanos, como inteiros. Isso mantém o
    parser do ESP32 simples (``atof``/``atoi`` sem ambiguidade).
    """
    if isinstance(value, bool):
        text = "1" if value else "0"
    elif isinstance(value, float):
        text = f"{value:.4f}"
    else:
        text = str(value)
    return Config(key=key, value=text).encode()


def encode_ping(seq: int) -> str:
    return Ping(seq=seq).encode()


# ---------------------------------------------------------------------------
# Decodificação
# ---------------------------------------------------------------------------


def split_frame(line: str) -> tuple[str, list[str]]:
    """Valida um quadro e devolve ``(tipo, campos)``.

    Aceita lixo antes do ``$`` (o ESP32 imprime mensagens de boot antes de o
    firmware assumir a serial) e ``\\r`` opcional no fim.
    """
    line = line.strip("\r\n")
    start = line.find(FRAME_START)
    if start < 0:
        raise ProtocolError(ErrorCode.BAD_FIELD_COUNT, "quadro sem '$'")
    line = line[start + 1 :]

    star = line.rfind(CHECKSUM_SEP)
    if star < 0:
        raise ProtocolError(ErrorCode.BAD_CHECKSUM, "quadro sem '*'")

    payload, received = line[:star], line[star + 1 :]
    if len(received) != 2:
        raise ProtocolError(
            ErrorCode.BAD_CHECKSUM, f"checksum com {len(received)} dígitos"
        )

    expected = checksum(payload)
    if received.upper() != expected:
        raise ProtocolError(
            ErrorCode.BAD_CHECKSUM, f"esperado {expected}, recebido {received.upper()}"
        )

    parts = payload.split(",")
    return parts[0], parts[1:]


def _require(fields: list[str], count: int, kind: str) -> None:
    if len(fields) != count:
        raise ProtocolError(
            ErrorCode.BAD_FIELD_COUNT,
            f"{kind} espera {count} campos, recebeu {len(fields)}",
        )


def _to_int(text: str, name: str) -> int:
    try:
        return int(text)
    except ValueError as exc:
        raise ProtocolError(ErrorCode.OUT_OF_RANGE, f"{name}='{text}'") from exc


def _to_float(text: str, name: str) -> float:
    try:
        return float(text)
    except ValueError as exc:
        raise ProtocolError(ErrorCode.OUT_OF_RANGE, f"{name}='{text}'") from exc


def parse(line: str) -> Message:
    """Decodifica uma linha completa em uma mensagem tipada."""
    kind, fields = split_frame(line)

    if kind == "TLM":
        # seq, t_ms, steer, throttle, src, d0..dN, vbat, flags
        if len(fields) < 8:
            raise ProtocolError(
                ErrorCode.BAD_FIELD_COUNT, f"TLM espera >= 8 campos, recebeu {len(fields)}"
            )
        distances = tuple(
            _to_int(f, f"d{i}") for i, f in enumerate(fields[5:-2])
        )
        try:
            flags = Flags(int(fields[-1], 16))
        except ValueError as exc:
            raise ProtocolError(ErrorCode.OUT_OF_RANGE, f"flags='{fields[-1]}'") from exc
        return Telemetry(
            seq=_to_int(fields[0], "seq"),
            t_ms=_to_int(fields[1], "t_ms"),
            steer=_to_float(fields[2], "steer"),
            throttle=_to_float(fields[3], "throttle"),
            source=Source(_to_int(fields[4], "src")),
            distances_mm=distances,
            vbat=_to_float(fields[-2], "vbat"),
            flags=flags,
        )

    if kind == "CMD":
        _require(fields, 4, "CMD")
        return Command(
            seq=_to_int(fields[0], "seq"),
            steer=_to_float(fields[1], "steer"),
            throttle=_to_float(fields[2], "throttle"),
            mode=Mode(_to_int(fields[3], "mode")),
        )

    if kind == "PONG":
        _require(fields, 2, "PONG")
        return Pong(seq=_to_int(fields[0], "seq"), t_ms=_to_int(fields[1], "t_ms"))

    if kind == "PING":
        _require(fields, 1, "PING")
        return Ping(seq=_to_int(fields[0], "seq"))

    if kind == "ARM":
        _require(fields, 1, "ARM")
        return Arm(armed=_to_int(fields[0], "armed") != 0)

    if kind == "CFG":
        _require(fields, 2, "CFG")
        return Config(key=fields[0], value=fields[1])

    if kind == "ACK":
        _require(fields, 2, "ACK")
        return Ack(kind=fields[0], detail=fields[1])

    if kind == "ERR":
        _require(fields, 2, "ERR")
        return Err(code=_to_int(fields[0], "code"), detail=fields[1])

    if kind == "LOG":
        if len(fields) < 2:
            raise ProtocolError(ErrorCode.BAD_FIELD_COUNT, "LOG espera >= 2 campos")
        return LogMessage(level=fields[0], text=",".join(fields[1:]))

    raise ProtocolError(ErrorCode.UNKNOWN_TYPE, kind)


# ---------------------------------------------------------------------------
# Parser incremental
# ---------------------------------------------------------------------------


@dataclass
class ParserStats:
    """Contadores do parser. Subiu erro? O enlace está ruim — investigue."""

    frames_ok: int = 0
    checksum_errors: int = 0
    malformed: int = 0
    unknown_type: int = 0
    dropped_bytes: int = 0

    @property
    def total_errors(self) -> int:
        return self.checksum_errors + self.malformed + self.unknown_type

    def as_dict(self) -> dict[str, int]:
        return {
            "frames_ok": self.frames_ok,
            "checksum_errors": self.checksum_errors,
            "malformed": self.malformed,
            "unknown_type": self.unknown_type,
            "dropped_bytes": self.dropped_bytes,
        }


class FrameParser:
    """Monta quadros a partir de um fluxo de bytes fragmentado.

    A serial entrega pedaços arbitrários: meia mensagem agora, o resto no
    próximo ``read()``. Este parser acumula até encontrar ``\\n``, tolera lixo
    e nunca levanta exceção no caminho normal — erros viram contadores em
    :attr:`stats`, para que uma linha corrompida não derrube o loop de
    controle.
    """

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.stats = ParserStats()

    def feed(self, data: bytes) -> list[Message]:
        """Consome bytes e devolve as mensagens completas encontradas."""
        if not data:
            return []
        self._buffer.extend(data)

        # Proteção contra fluxo sem terminador (ruído puro na linha).
        if len(self._buffer) > _MAX_BUFFER:
            excess = len(self._buffer) - MAX_FRAME_LEN
            del self._buffer[:excess]
            self.stats.dropped_bytes += excess

        messages: list[Message] = []
        while True:
            index = self._buffer.find(b"\n")
            if index < 0:
                break
            raw = bytes(self._buffer[:index])
            del self._buffer[: index + 1]
            message = self._parse_line(raw)
            if message is not None:
                messages.append(message)
        return messages

    def _parse_line(self, raw: bytes) -> Message | None:
        try:
            line = raw.decode("ascii", errors="strict")
        except UnicodeDecodeError:
            self.stats.malformed += 1
            self.stats.dropped_bytes += len(raw)
            return None

        if FRAME_START not in line:
            # Linha sem '$': banner de boot do ESP32, ruído. Silenciosamente ignorada.
            self.stats.dropped_bytes += len(raw)
            return None

        try:
            message = parse(line)
        except ProtocolError as exc:
            if exc.code is ErrorCode.BAD_CHECKSUM:
                self.stats.checksum_errors += 1
            elif exc.code is ErrorCode.UNKNOWN_TYPE:
                self.stats.unknown_type += 1
            else:
                self.stats.malformed += 1
            return None

        self.stats.frames_ok += 1
        return message

    def reset(self) -> None:
        """Limpa o buffer (usar após reconectar a porta)."""
        self._buffer.clear()


# ---------------------------------------------------------------------------
# Mapeamento config/vehicle.yaml -> chaves de $CFG
# ---------------------------------------------------------------------------

#: Cada entrada é ``(chave do protocolo, caminho no vehicle.yaml, conversor)``.
#: Usado por :func:`build_config_frames` no handshake.
CONFIG_MAP: tuple[tuple[str, str, type], ...] = (
    ("steer_center_us", "steering.center_us", int),
    ("steer_min_us", "steering.min_us", int),
    ("steer_max_us", "steering.max_us", int),
    ("steer_invert", "steering.invert", bool),
    ("thr_neutral_us", "throttle.neutral_us", int),
    ("thr_min_us", "throttle.min_us", int),
    ("thr_max_us", "throttle.max_us", int),
    ("thr_deadband", "throttle.deadband", float),
    ("thr_limit_rev", "throttle.limit_reverse", float),
    ("slew_steer", "steering.slew_rate_per_s", float),
    ("slew_thr", "throttle.slew_rate_per_s", float),
    ("timeout_ms", "safety.command_timeout_ms", int),
    ("obst_stop_mm", "safety.obstacle_stop_mm", int),
    ("obst_slow_mm", "safety.obstacle_slow_mm", int),
    ("tlm_hz", "safety.telemetry_hz", int),
)


def _dig(data: dict, path: str) -> object:
    node: object = data
    for part in path.split("."):
        if not isinstance(node, dict) or part not in node:
            raise KeyError(f"chave ausente em vehicle.yaml: {path}")
        node = node[part]
    return node


def build_config_frames(
    vehicle: dict,
    *,
    throttle_limit: float,
    rc_enable: bool = False,
    extra: Iterable[tuple[str, object]] = (),
) -> list[tuple[str, str]]:
    """Monta a lista de quadros ``$CFG`` do handshake.

    Args:
        vehicle: conteúdo de ``config/vehicle.yaml``.
        throttle_limit: teto de aceleração para frente neste modo de operação
            (``limit_collect``, ``limit_auto`` ou ``limit_race``).
        rc_enable: habilita o rádio RC. **Sempre False em prova oficial** —
            o regulamento proíbe controle externo.
        extra: pares adicionais ``(chave, valor)``.

    Returns:
        Lista de ``(chave, quadro codificado)``, na ordem de envio.
    """
    frames: list[tuple[str, str]] = []
    for key, path, kind in CONFIG_MAP:
        raw = _dig(vehicle, path)
        value = kind(raw) if kind is not bool else bool(raw)
        frames.append((key, encode_config(key, value)))

    frames.append(("thr_limit_fwd", encode_config("thr_limit_fwd", float(throttle_limit))))
    frames.append(("rc_enable", encode_config("rc_enable", bool(rc_enable))))
    for key, value in extra:
        frames.append((key, encode_config(key, value)))
    return frames


__all__ = [
    "PROTOCOL_VERSION",
    "MAX_FRAME_LEN",
    "Ack",
    "Arm",
    "Command",
    "Config",
    "Err",
    "ErrorCode",
    "Flags",
    "FrameParser",
    "LogMessage",
    "Message",
    "Mode",
    "ParserStats",
    "Ping",
    "Pong",
    "ProtocolError",
    "Source",
    "Telemetry",
    "UNSAFE_FOR_TRAINING",
    "build_config_frames",
    "checksum",
    "clamp",
    "encode_arm",
    "encode_command",
    "encode_config",
    "encode_frame",
    "encode_ping",
    "parse",
    "split_frame",
]
