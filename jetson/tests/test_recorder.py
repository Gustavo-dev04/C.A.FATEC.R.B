"""Testes do gravador de sessões.

Roda sem OpenCV e sem hardware: o gravador aceita um módulo ``cv2`` injetado,
o que permite exercitar toda a lógica de filtragem, fila e escrita em disco.
"""

from __future__ import annotations

import json
import time

import pytest

from robocar.capture.recorder import SessionRecorder
from robocar.capture.schema import RECORDS_FILENAME, SessionMeta, read_records
from robocar.comms import protocol as p
from robocar.config import Config


class FakeCv2:
    """Substituto de ``cv2`` com o mínimo que o gravador usa."""

    IMWRITE_JPEG_QUALITY = 1

    def __init__(self) -> None:
        self.calls = 0

    def imencode(self, ext, frame, params=None):
        self.calls += 1

        class _Buffer:
            def tobytes(self_inner):
                return b"\xff\xd8fake-jpeg\xff\xd9"

        return True, _Buffer()


@pytest.fixture
def config():
    return Config(
        {
            "capture": {
                "image": {"quality": 90, "filename_pattern": "{index:06d}.jpg"},
                "labels": {"max_telemetry_age_ms": 60},
                "filters": {
                    "skip_when_stopped": True,
                    "min_throttle": 0.03,
                    "warmup_s": 0.0,
                },
                "writer": {"queue_size": 64, "workers": 1, "fsync_every": 5},
            }
        }
    )


@pytest.fixture
def recorder(tmp_path, config):
    meta = SessionMeta(session_id="teste")
    rec = SessionRecorder(tmp_path, meta, config, cv2_module=FakeCv2()).start()
    yield rec
    if rec._records_file is not None:
        rec.stop()


def _telemetry(**kwargs):
    base = {
        "seq": 1,
        "t_ms": 1000,
        "steer": 0.35,
        "throttle": 0.25,
        "source": p.Source.RC,
        "distances_mm": (1200, 900, 1100),
        "vbat": 7.8,
        "flags": p.Flags.ARMED | p.Flags.RC_VALID,
    }
    base.update(kwargs)
    return p.Telemetry(**base)


def _drain(recorder, timeout=2.0):
    """Espera a fila de escrita esvaziar e força o flush do JSONL."""
    deadline = time.monotonic() + timeout
    while recorder.queue_depth and time.monotonic() < deadline:
        time.sleep(0.01)
    time.sleep(0.05)
    recorder.flush()


# --- caminho feliz ---------------------------------------------------------


def test_grava_frame_valido(recorder, tmp_path):
    assert recorder.submit("frame", time.monotonic_ns(), _telemetry(), 5.0) is True
    _drain(recorder)

    assert recorder.frames_written == 1
    assert (tmp_path / "images" / "000000.jpg").exists()

    registros = read_records(tmp_path)
    assert len(registros) == 1
    assert registros[0].steer == pytest.approx(0.35)
    assert registros[0].image == "images/000000.jpg"


def test_rotulo_vem_da_telemetria_nao_do_comando(recorder, tmp_path):
    """ADR 0002: o rótulo é o valor aplicado, não o comandado."""
    telemetry = _telemetry(steer=0.10, throttle=0.15)
    recorder.submit(
        "frame",
        time.monotonic_ns(),
        telemetry,
        5.0,
        command=(0.90, 0.80),  # o que o Jetson pediu
    )
    _drain(recorder)

    registro = read_records(tmp_path)[0]
    assert registro.steer == pytest.approx(0.10)   # aplicado
    assert registro.throttle == pytest.approx(0.15)
    assert registro.cmd_steer == pytest.approx(0.90)  # só diagnóstico
    assert registro.cmd_throttle == pytest.approx(0.80)


def test_indices_sao_sequenciais(recorder, tmp_path):
    for _ in range(5):
        recorder.submit("frame", time.monotonic_ns(), _telemetry(), 1.0)
    _drain(recorder)
    assert [r.index for r in read_records(tmp_path)] == [0, 1, 2, 3, 4]


def test_metadados_da_telemetria_sao_preservados(recorder, tmp_path):
    telemetry = _telemetry(distances_mm=(500, 0, 1800), vbat=7.15, t_ms=42)
    recorder.submit("frame", time.monotonic_ns(), telemetry, 12.5)
    _drain(recorder)

    registro = read_records(tmp_path)[0]
    assert registro.distances_mm == [500, 0, 1800]
    assert registro.vbat == pytest.approx(7.15)
    assert registro.t_mcu_ms == 42
    assert registro.telemetry_age_ms == pytest.approx(12.5)
    assert registro.source == int(p.Source.RC)


# --- filtros ---------------------------------------------------------------


def test_descarta_sem_telemetria(recorder):
    assert recorder.submit("frame", time.monotonic_ns(), None, float("inf")) is False
    assert recorder.stats.skipped_stale == 1
    assert recorder.frames_written == 0


def test_descarta_telemetria_velha(recorder):
    # max_telemetry_age_ms = 60
    assert recorder.submit("frame", time.monotonic_ns(), _telemetry(), 80.0) is False
    assert recorder.stats.skipped_stale == 1


def test_aceita_telemetria_no_limite_da_idade(recorder):
    assert recorder.submit("frame", time.monotonic_ns(), _telemetry(), 60.0) is True


def test_descarta_carro_parado(recorder):
    assert recorder.submit("frame", time.monotonic_ns(), _telemetry(throttle=0.0), 1.0) is False
    assert recorder.stats.skipped_stopped == 1


@pytest.mark.parametrize(
    "flag", [p.Flags.FAILSAFE, p.Flags.OBSTACLE_STOP, p.Flags.BATT_CRITICAL]
)
def test_descarta_estados_inseguros(recorder, flag):
    telemetry = _telemetry(flags=p.Flags.ARMED | flag)
    assert recorder.submit("frame", time.monotonic_ns(), telemetry, 1.0) is False
    assert recorder.stats.skipped_unsafe == 1


def test_aceita_obstaculo_apenas_lento(recorder):
    # OBSTACLE_SLOW limita a velocidade mas o comando ainda é do piloto.
    telemetry = _telemetry(flags=p.Flags.ARMED | p.Flags.OBSTACLE_SLOW)
    assert recorder.submit("frame", time.monotonic_ns(), telemetry, 1.0) is True


def test_aquecimento_descarta_frames_iniciais(tmp_path, config):
    config.data["capture"]["filters"]["warmup_s"] = 10.0
    rec = SessionRecorder(tmp_path, SessionMeta(session_id="x"), config, cv2_module=FakeCv2())
    rec.start()
    try:
        assert rec.submit("frame", time.monotonic_ns(), _telemetry(), 1.0) is False
        assert rec.stats.skipped_warmup == 1
    finally:
        rec.stop()


def test_frames_descartados_nao_consomem_indice(recorder, tmp_path):
    recorder.submit("frame", time.monotonic_ns(), _telemetry(), 1.0)          # index 0
    recorder.submit("frame", time.monotonic_ns(), _telemetry(throttle=0), 1.0)  # descartado
    recorder.submit("frame", time.monotonic_ns(), _telemetry(), 1.0)          # index 1
    _drain(recorder)
    # Sem buracos: cada índice tem uma imagem correspondente.
    assert [r.index for r in read_records(tmp_path)] == [0, 1]


# --- encerramento ----------------------------------------------------------


def test_stop_drena_a_fila_e_grava_metadados(tmp_path, config):
    rec = SessionRecorder(tmp_path, SessionMeta(session_id="x"), config, cv2_module=FakeCv2())
    rec.start()
    for _ in range(20):
        rec.submit("frame", time.monotonic_ns(), _telemetry(), 1.0)
    meta = rec.stop()

    assert meta.frame_count == 20
    assert len(read_records(tmp_path)) == 20
    assert meta.duration_s >= 0
    assert meta.ended_at != ""


def test_metadados_registram_os_descartes(tmp_path, config):
    rec = SessionRecorder(tmp_path, SessionMeta(session_id="x"), config, cv2_module=FakeCv2())
    rec.start()
    rec.submit("frame", time.monotonic_ns(), _telemetry(), 1.0)
    rec.submit("frame", time.monotonic_ns(), _telemetry(throttle=0.0), 1.0)
    rec.submit("frame", time.monotonic_ns(), None, float("inf"))
    meta = rec.stop()

    assert meta.frame_count == 1
    assert meta.skipped_stopped == 1
    assert meta.skipped_stale == 1
    assert meta.yield_ratio == pytest.approx(1 / 3)


def test_context_manager(tmp_path, config):
    with SessionRecorder(tmp_path, SessionMeta(session_id="x"), config, cv2_module=FakeCv2()) as rec:
        rec.submit("frame", time.monotonic_ns(), _telemetry(), 1.0)
    assert (tmp_path / RECORDS_FILENAME).exists()


def test_jsonl_tem_uma_linha_por_frame(tmp_path, config):
    rec = SessionRecorder(tmp_path, SessionMeta(session_id="x"), config, cv2_module=FakeCv2())
    rec.start()
    for _ in range(7):
        rec.submit("frame", time.monotonic_ns(), _telemetry(), 1.0)
    rec.stop()

    linhas = (tmp_path / RECORDS_FILENAME).read_text().strip().split("\n")
    assert len(linhas) == 7
    for linha in linhas:
        json.loads(linha)  # cada linha precisa ser JSON válido isoladamente
