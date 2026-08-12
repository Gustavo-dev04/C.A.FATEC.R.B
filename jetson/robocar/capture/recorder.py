"""Gravador de sessões: casa frame com telemetria e escreve em disco.

Arquitetura em três threads:

    [captura]  --fila-->  [N workers de escrita]  -->  images/*.jpg
         |                                              records.jsonl
         +--> decide se o frame vale, monta o Record

Codificar JPEG custa 5–15 ms por frame. Fazer isso na thread de captura
derrubaria a taxa e, pior, introduziria jitter: os frames deixariam de ser
igualmente espaçados no tempo e o dataset ficaria com amostragem irregular.
Daí a fila.

A fila é **limitada** de propósito. Se o disco não acompanha, é melhor
descartar frames e contabilizar o descarte (`dropped_frames`) do que crescer
memória até o processo morrer no meio da coleta.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from ..comms import protocol as p
from .schema import IMAGES_DIRNAME, RECORDS_FILENAME, Record, SessionMeta

log = logging.getLogger(__name__)


@dataclass
class RecorderStats:
    """Contadores da sessão. Espelhados em ``session.json`` ao final."""

    written: int = 0
    dropped_queue_full: int = 0
    skipped_stopped: int = 0
    skipped_stale: int = 0
    skipped_unsafe: int = 0
    skipped_warmup: int = 0
    write_errors: int = 0

    @property
    def considered(self) -> int:
        return (
            self.written
            + self.dropped_queue_full
            + self.skipped_stopped
            + self.skipped_stale
            + self.skipped_unsafe
            + self.skipped_warmup
        )

    def summary(self) -> str:
        return (
            f"gravados={self.written} descartados_fila={self.dropped_queue_full} "
            f"parado={self.skipped_stopped} telemetria_velha={self.skipped_stale} "
            f"inseguro={self.skipped_unsafe} aquecimento={self.skipped_warmup} "
            f"erros_escrita={self.write_errors}"
        )


class SessionRecorder:
    """Grava uma sessão de coleta.

    Não sabe de onde vem o comando (rádio RC, gamepad ou modelo): recebe frame
    + telemetria e cuida de decidir, rotular e persistir. Ver ADR 0002.
    """

    def __init__(
        self,
        session_dir: Path,
        meta: SessionMeta,
        config: Any,
        *,
        cv2_module: Any = None,
    ) -> None:
        self.session_dir = Path(session_dir)
        self.meta = meta
        self.stats = RecorderStats()

        self._images_dir = self.session_dir / IMAGES_DIRNAME
        self._records_path = self.session_dir / RECORDS_FILENAME

        # --- parâmetros de filtragem ---
        self._quality = config.get_int("capture.image.quality", 90)
        self._pattern = config.get_str(
            "capture.image.filename_pattern", "{index:06d}.jpg"
        )
        self._max_age_ms = config.get_float("capture.labels.max_telemetry_age_ms", 60.0)
        self._skip_stopped = config.get_bool("capture.filters.skip_when_stopped", True)
        self._min_throttle = config.get_float("capture.filters.min_throttle", 0.03)
        self._warmup_s = config.get_float("capture.filters.warmup_s", 1.5)

        queue_size = config.get_int("capture.writer.queue_size", 256)
        self._workers = config.get_int("capture.writer.workers", 2)
        self._fsync_every = config.get_int("capture.writer.fsync_every", 200)

        self._cv2 = cv2_module
        self._queue: queue.Queue = queue.Queue(maxsize=queue_size)
        self._threads: list[threading.Thread] = []
        self._records_lock = threading.Lock()
        self._records_file = None
        self._index = 0
        self._started_ns = 0
        self._stop = threading.Event()
        self._since_flush = 0

    # -- ciclo de vida -----------------------------------------------------

    def start(self) -> SessionRecorder:
        if self._cv2 is None:
            import cv2  # type: ignore[import-not-found]

            self._cv2 = cv2

        self._images_dir.mkdir(parents=True, exist_ok=True)
        self._records_file = self._records_path.open("a", encoding="utf-8")
        self._started_ns = time.monotonic_ns()
        self._stop.clear()

        for i in range(max(1, self._workers)):
            thread = threading.Thread(
                target=self._writer_loop, name=f"robocar-writer-{i}", daemon=True
            )
            thread.start()
            self._threads.append(thread)

        log.info("gravando em %s", self.session_dir)
        return self

    def stop(self, *, timeout: float = 10.0) -> SessionMeta:
        """Drena a fila, fecha os arquivos e devolve os metadados finais.

        Sempre chame isto (ou use o ``with``): sem drenar a fila, os últimos
        frames capturados nunca chegam ao disco.
        """
        deadline = time.monotonic() + timeout
        while not self._queue.empty() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not self._queue.empty():
            log.warning(
                "encerrando com %d frames ainda na fila", self._queue.qsize()
            )

        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=2.0)
        self._threads.clear()

        if self._records_file is not None:
            self._records_file.flush()
            self._records_file.close()
            self._records_file = None

        duration = (time.monotonic_ns() - self._started_ns) / 1e9
        self.meta.ended_at = _now_iso()
        self.meta.duration_s = round(duration, 2)
        self.meta.frame_count = self.stats.written
        self.meta.dropped_frames = self.stats.dropped_queue_full
        self.meta.skipped_stopped = self.stats.skipped_stopped
        self.meta.skipped_stale = self.stats.skipped_stale
        self.meta.skipped_unsafe = self.stats.skipped_unsafe

        from .schema import write_session_meta

        write_session_meta(self.session_dir, self.meta)
        log.info("sessão encerrada: %s", self.stats.summary())
        return self.meta

    def __enter__(self) -> SessionRecorder:
        return self.start()

    def __exit__(self, *exc_info: object) -> None:
        self.stop()

    # -- captura -----------------------------------------------------------

    def submit(
        self,
        frame: Any,
        t_host_ns: int,
        telemetry: p.Telemetry | None,
        telemetry_age_ms: float,
        *,
        command: tuple[float, float] | None = None,
    ) -> bool:
        """Considera um frame para gravação. Devolve True se foi enfileirado.

        A ordem dos filtros é do mais barato para o mais caro, para não gastar
        trabalho com um frame que vai ser descartado de qualquer jeito.
        """
        elapsed_s = (t_host_ns - self._started_ns) / 1e9
        if elapsed_s < self._warmup_s:
            self.stats.skipped_warmup += 1
            return False

        if telemetry is None or telemetry_age_ms > self._max_age_ms:
            # Sem rótulo confiável não há amostra. Ver ADR 0002.
            self.stats.skipped_stale += 1
            return False

        if not telemetry.usable_for_training:
            # Failsafe, corte por obstáculo ou bateria crítica: o valor
            # aplicado não é a intenção do piloto.
            self.stats.skipped_unsafe += 1
            return False

        if self._skip_stopped and abs(telemetry.throttle) < self._min_throttle:
            # Carro parado gera milhares de frames idênticos que ensinariam a
            # rede a ficar parada — o comportamento mais fácil de aprender e o
            # menos útil na pista.
            self.stats.skipped_stopped += 1
            return False

        index = self._index
        self._index += 1

        record = Record(
            index=index,
            image=f"{IMAGES_DIRNAME}/{self._pattern.format(index=index)}",
            t_host_ns=t_host_ns,
            steer=telemetry.steer,
            throttle=telemetry.throttle,
            t_mcu_ms=telemetry.t_ms,
            telemetry_age_ms=round(telemetry_age_ms, 2),
            source=int(telemetry.source),
            flags=int(telemetry.flags),
            distances_mm=list(telemetry.distances_mm),
            vbat=telemetry.vbat,
            cmd_steer=command[0] if command else None,
            cmd_throttle=command[1] if command else None,
        )

        try:
            self._queue.put_nowait((frame, record))
            return True
        except queue.Full:
            self._index -= 1  # devolve o índice: este frame não vai existir
            self.stats.dropped_queue_full += 1
            if self.stats.dropped_queue_full % 50 == 1:
                log.warning(
                    "fila de escrita cheia (%d descartes acumulados) — o disco não "
                    "acompanha a taxa de captura. Reduza capture.image.quality ou "
                    "capture.rate_hz, ou grave em NVMe em vez de cartão SD.",
                    self.stats.dropped_queue_full,
                )
            return False

    # -- escrita -----------------------------------------------------------

    def _writer_loop(self) -> None:
        while not self._stop.is_set() or not self._queue.empty():
            try:
                frame, record = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self._write_one(frame, record)
            except Exception as exc:  # pragma: no cover - depende de I/O real
                self.stats.write_errors += 1
                log.error("falha ao gravar frame %d: %s", record.index, exc)
            finally:
                self._queue.task_done()

    def _write_one(self, frame: Any, record: Record) -> None:
        image_path = self.session_dir / record.image
        ok, buffer = self._cv2.imencode(
            ".jpg", frame, [int(self._cv2.IMWRITE_JPEG_QUALITY), self._quality]
        )
        if not ok:
            raise RuntimeError("cv2.imencode falhou")
        image_path.write_bytes(buffer.tobytes())

        line = record.to_json() + "\n"
        with self._records_lock:
            if self._records_file is None:
                return
            self._records_file.write(line)
            self.stats.written += 1
            self._since_flush += 1
            if self._since_flush >= self._fsync_every:
                # Flush periódico: uma queda de energia custa no máximo
                # `fsync_every` registros, não a sessão inteira.
                self._records_file.flush()
                self._since_flush = 0

    def flush(self) -> None:
        """Força para o disco os registros ainda no buffer do arquivo.

        Entre um flush e outro, ``fsync_every`` registros vivem só na memória
        do processo. Chame isto em pontos de checkpoint de uma sessão longa —
        e o :meth:`stop` já chama por você no encerramento normal.
        """
        with self._records_lock:
            if self._records_file is not None:
                self._records_file.flush()
                self._since_flush = 0

    # -- diagnóstico -------------------------------------------------------

    @property
    def queue_depth(self) -> int:
        return self._queue.qsize()

    @property
    def frames_written(self) -> int:
        return self.stats.written


def _now_iso() -> str:
    from datetime import datetime

    return datetime.now().astimezone().isoformat(timespec="seconds")


__all__ = ["RecorderStats", "SessionRecorder"]
