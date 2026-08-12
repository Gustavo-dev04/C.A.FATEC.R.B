"""Enlace serial com o ESP32: thread de leitura, handshake e watchdog.

Uso típico::

    with SerialLink("/dev/ttyUSB0") as link:
        link.handshake(config_frames)
        link.arm(True)
        while running:
            link.send_command(steer, throttle, Mode.AUTO)
            tlm = link.latest_telemetry()

``pyserial`` é importado de forma preguiçosa para que o resto do pacote
(protocolo, esquema do dataset) continue utilizável em máquinas sem a
dependência instalada — como a CI.
"""

from __future__ import annotations

import contextlib
import logging
import threading
import time
from dataclasses import dataclass
from typing import Callable

from . import protocol as p

log = logging.getLogger(__name__)

DEFAULT_BAUDRATE = 115200
DEFAULT_PORT_CANDIDATES = (
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
    "/dev/ttyACM0",
    "/dev/ttyACM1",
)


class LinkError(RuntimeError):
    """Falha de enlace com o ESP32."""


@dataclass
class LinkStats:
    commands_sent: int = 0
    telemetry_received: int = 0
    reconnects: int = 0
    last_telemetry_ns: int = 0
    write_errors: int = 0


def find_port(candidates: tuple[str, ...] = DEFAULT_PORT_CANDIDATES) -> str:
    """Procura a porta do ESP32.

    Tenta primeiro os apelidos estáveis criados pela regra udev
    (``tools/udev/99-robocar.rules``) e depois os nomes genéricos — que mudam
    de ordem quando há mais de um dispositivo USB conectado.
    """
    import glob
    import os

    for path in ("/dev/robocar-esp32", *candidates):
        if os.path.exists(path):
            return path

    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        found = sorted(glob.glob(pattern))
        if found:
            return found[0]

    raise LinkError(
        "nenhuma porta serial encontrada. Conecte o ESP32 e confira "
        "`ls /dev/ttyUSB* /dev/ttyACM*`."
    )


class SerialLink:
    """Enlace com o ESP32, com leitura em thread e reconexão automática.

    A telemetria mais recente fica em cache e é lida sem bloqueio pelo loop de
    controle: em um carro, é sempre melhor agir com um dado de 20 ms atrás do
    que esperar por um dado novo.
    """

    def __init__(
        self,
        port: str | None = None,
        baudrate: int = DEFAULT_BAUDRATE,
        *,
        timeout: float = 0.05,
        auto_reconnect: bool = True,
        on_message: Callable[[p.Message], None] | None = None,
    ) -> None:
        self.port = port or find_port()
        self.baudrate = baudrate
        self.timeout = timeout
        self.auto_reconnect = auto_reconnect
        self._on_message = on_message

        self._serial = None
        self._parser = p.FrameParser()
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._telemetry: p.Telemetry | None = None
        self._telemetry_ns: int = 0
        self._pending: dict[int, p.Pong] = {}
        self._acks: list[p.Ack] = []
        self._errors: list[p.Err] = []
        self._seq = 0
        self.stats = LinkStats()

    # -- ciclo de vida -----------------------------------------------------

    def open(self) -> SerialLink:
        try:
            import serial  # type: ignore[import-not-found]
        except ImportError as exc:  # pragma: no cover - ambiente sem hardware
            raise LinkError(
                "pyserial não instalado. Rode: pip install -r jetson/requirements.txt"
            ) from exc

        try:
            self._serial = serial.Serial(
                self.port, self.baudrate, timeout=self.timeout
            )
        except Exception as exc:  # pragma: no cover
            raise LinkError(f"não foi possível abrir {self.port}: {exc}") from exc

        # O ESP32 reinicia ao abrir a serial (DTR) e cospe o banner de boot.
        # Esperar aqui evita interpretar esse lixo como quadro corrompido.
        time.sleep(0.3)
        self._serial.reset_input_buffer()
        self._parser.reset()

        self._stop.clear()
        self._thread = threading.Thread(
            target=self._read_loop, name="robocar-serial", daemon=True
        )
        self._thread.start()
        log.info("enlace aberto em %s @ %d baud", self.port, self.baudrate)
        return self

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._serial is not None:
            try:
                # Última cortesia: manda o carro para o neutro e desarma.
                self._write(p.encode_command(self._seq, 0.0, 0.0, p.Mode.IDLE))
                self._write(p.encode_arm(False))
                self._serial.flush()
            except Exception:  # pragma: no cover - a porta pode já ter sumido
                pass
            self._serial.close()
            self._serial = None
        log.info("enlace fechado (%s)", self.stats)

    def __enter__(self) -> SerialLink:
        return self.open()

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    # -- leitura -----------------------------------------------------------

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            try:
                if self._serial is None:
                    raise OSError("porta fechada")
                data = self._serial.read(512)
            except Exception as exc:  # pragma: no cover - depende de hardware
                log.warning("erro de leitura: %s", exc)
                if not self.auto_reconnect:
                    break
                self._reconnect()
                continue

            if not data:
                continue

            for message in self._parser.feed(data):
                self._dispatch(message)

    def _dispatch(self, message: p.Message) -> None:
        now = time.monotonic_ns()
        if isinstance(message, p.Telemetry):
            with self._lock:
                self._telemetry = message
                self._telemetry_ns = now
            self.stats.telemetry_received += 1
            self.stats.last_telemetry_ns = now
        elif isinstance(message, p.Pong):
            with self._lock:
                self._pending[message.seq] = message
        elif isinstance(message, p.Ack):
            with self._lock:
                self._acks.append(message)
        elif isinstance(message, p.Err):
            with self._lock:
                self._errors.append(message)
            log.warning("ESP32 reportou erro %s: %s", message.code, message.detail)
        elif isinstance(message, p.LogMessage):
            log.info("[esp32/%s] %s", message.level, message.text)

        if self._on_message is not None:
            self._on_message(message)

    def _reconnect(self) -> None:  # pragma: no cover - depende de hardware
        if self._serial is not None:
            with contextlib.suppress(Exception):
                self._serial.close()
            self._serial = None
        time.sleep(0.5)
        try:
            self.open_serial_only()
            self.stats.reconnects += 1
            log.info("reconectado a %s", self.port)
        except Exception as exc:
            log.warning("falha ao reconectar: %s", exc)
            time.sleep(1.0)

    def open_serial_only(self) -> None:  # pragma: no cover - depende de hardware
        """Reabre a porta sem recriar a thread de leitura."""
        import serial  # type: ignore[import-not-found]

        self._serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        time.sleep(0.3)
        self._serial.reset_input_buffer()
        self._parser.reset()

    # -- escrita -----------------------------------------------------------

    def _write(self, frame: str) -> None:
        if self._serial is None:
            raise LinkError("enlace fechado")
        self._serial.write(frame.encode("ascii"))

    def send_raw(self, frame: str) -> bool:
        try:
            self._write(frame)
            return True
        except Exception as exc:  # pragma: no cover - depende de hardware
            self.stats.write_errors += 1
            log.warning("erro de escrita: %s", exc)
            return False

    def send_command(
        self, steer: float, throttle: float, mode: p.Mode = p.Mode.AUTO
    ) -> int:
        """Envia ``$CMD``. Devolve o ``seq`` usado.

        Precisa ser chamado a pelo menos ``1000/timeout_ms`` Hz (4 Hz com o
        padrão de 250 ms) ou o ESP32 entra em failsafe. Na prática, chame a
        50 Hz.
        """
        self._seq = (self._seq + 1) % p.SEQ_MODULO
        if self.send_raw(p.encode_command(self._seq, steer, throttle, mode)):
            self.stats.commands_sent += 1
        return self._seq

    def arm(self, armed: bool) -> bool:
        """Arma/desarma a tração e espera o ``$ACK``."""
        self.send_raw(p.encode_arm(armed))
        return self._wait_ack("ARM", timeout=0.5)

    def stop(self) -> None:
        """Neutro imediato. Chamável de qualquer contexto, inclusive handler."""
        self.send_raw(p.encode_command(self._seq, 0.0, 0.0, p.Mode.IDLE))

    # -- consultas ---------------------------------------------------------

    def latest_telemetry(self) -> p.Telemetry | None:
        with self._lock:
            return self._telemetry

    def telemetry_with_age(self) -> tuple[p.Telemetry | None, float]:
        """Devolve ``(telemetria, idade_em_ms)``.

        A idade é o que decide se a amostra vira rótulo de treino
        (``capture.labels.max_telemetry_age_ms``).
        """
        with self._lock:
            telemetry = self._telemetry
            stamp = self._telemetry_ns
        if telemetry is None:
            return None, float("inf")
        return telemetry, (time.monotonic_ns() - stamp) / 1e6

    def ping(self, timeout: float = 1.0) -> float | None:
        """Mede o tempo de ida e volta em ms. ``None`` se não houve resposta."""
        self._seq = (self._seq + 1) % p.SEQ_MODULO
        seq = self._seq
        with self._lock:
            self._pending.pop(seq, None)
        start = time.monotonic()
        if not self.send_raw(p.encode_ping(seq)):
            return None
        while time.monotonic() - start < timeout:
            with self._lock:
                if seq in self._pending:
                    self._pending.pop(seq)
                    return (time.monotonic() - start) * 1000.0
            time.sleep(0.002)
        return None

    def _wait_ack(self, kind: str, timeout: float = 0.5) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                for ack in self._acks:
                    if ack.kind == kind:
                        self._acks.remove(ack)
                        return True
            time.sleep(0.002)
        return False

    def drain_errors(self) -> list[p.Err]:
        with self._lock:
            errors, self._errors = self._errors, []
        return errors

    # -- handshake ---------------------------------------------------------

    def handshake(
        self,
        config_frames: list[tuple[str, str]],
        *,
        ping_timeout: float = 2.0,
        ack_timeout: float = 0.5,
    ) -> None:
        """Executa o handshake completo descrito em docs/03-protocolo-serial.md.

        Levanta :class:`LinkError` se qualquer ``$CFG`` não for confirmado.
        Seguir com calibração parcial é pior do que não calibrar: o carro
        parece funcionar e se comporta de um jeito que ninguém consegue
        explicar.
        """
        rtt = self.ping(timeout=ping_timeout)
        if rtt is None:
            raise LinkError(
                f"ESP32 não respondeu ao $PING em {self.port}. "
                "Confira firmware, cabo e baud rate."
            )
        log.info("ESP32 respondeu em %.2f ms", rtt)

        for key, frame in config_frames:
            self.send_raw(frame)
            if not self._wait_ack("CFG", timeout=ack_timeout):
                raise LinkError(f"ESP32 não confirmou $CFG para '{key}'")
            log.debug("cfg ok: %s", key)

        self.send_raw(p.encode_arm(False))
        self._wait_ack("ARM", timeout=ack_timeout)

        # Confirma que a telemetria já reflete a configuração recebida.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            telemetry = self.latest_telemetry()
            if telemetry is not None and not (telemetry.flags & p.Flags.CFG_PENDING):
                log.info("handshake concluído")
                return
            time.sleep(0.02)
        raise LinkError(
            "ESP32 segue com CFG_PENDING após o handshake — configuração não aplicada."
        )

    @property
    def parser_stats(self) -> p.ParserStats:
        return self._parser.stats


__all__ = ["DEFAULT_BAUDRATE", "LinkError", "LinkStats", "SerialLink", "find_port"]
