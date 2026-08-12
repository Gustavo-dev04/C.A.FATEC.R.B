"""Testes do controle de taxa do loop."""

from __future__ import annotations

import time

import pytest

from robocar.util.rate import LoopMonitor, RateLimiter


def test_hz_invalido_levanta():
    for hz in (0, -5):
        with pytest.raises(ValueError):
            RateLimiter(hz)


def test_mantem_a_taxa_aproximada():
    limiter = RateLimiter(100.0)
    inicio = time.monotonic()
    for _ in range(20):
        limiter.sleep()
    decorrido = time.monotonic() - inicio
    # 20 ticks a 100 Hz = 0,2 s. Folga generosa para máquina de CI carregada.
    assert 0.15 < decorrido < 0.6


def test_nao_acumula_deriva_apos_atraso():
    limiter = RateLimiter(50.0)
    limiter.sleep()
    time.sleep(0.1)  # estoura 5 períodos de propósito

    inicio = time.monotonic()
    limiter.sleep()
    # Deve realinhar ao próximo tick, não disparar 5 iterações instantâneas
    # tentando "recuperar" o atraso.
    assert time.monotonic() - inicio < 0.05
    assert limiter.late_count >= 1


def test_late_ratio():
    limiter = RateLimiter(1000.0)
    for _ in range(5):
        limiter.sleep()
    assert 0.0 <= limiter.late_ratio <= 1.0


def test_monitor_sem_amostras_nao_divide_por_zero():
    monitor = LoopMonitor()
    assert monitor.hz == 0.0
    assert monitor.mean_ms == 0.0
    assert monitor.p95_ms == 0.0
    assert monitor.max_ms == 0.0


def test_monitor_mede_periodo():
    monitor = LoopMonitor()
    for _ in range(5):
        monitor.tick()
        time.sleep(0.01)
    assert monitor.mean_ms > 5.0
    assert monitor.max_ms >= monitor.p95_ms >= 0.0


def test_monitor_limita_a_janela():
    monitor = LoopMonitor(window=10)
    for _ in range(50):
        monitor.tick()
    assert len(monitor._samples) <= 10
