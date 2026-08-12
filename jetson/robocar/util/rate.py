"""Controle de taxa e medição de desempenho do loop.

Um loop de coleta precisa de duas coisas: rodar na frequência pedida e avisar
quando **não** está conseguindo. Se a taxa cai de 20 para 12 Hz, o dataset fica
com amostragem irregular e ninguém percebe até o treino sair estranho.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field


class RateLimiter:
    """Mantém um loop numa frequência alvo.

    Usa relógio monotônico e alvo absoluto (não ``sleep(1/hz)`` a cada volta),
    de modo que atrasos pontuais não acumulam deriva.
    """

    def __init__(self, hz: float) -> None:
        if hz <= 0:
            raise ValueError(f"hz precisa ser positivo, recebi {hz}")
        self.hz = hz
        self.period_ns = int(1e9 / hz)
        self._next_ns = time.monotonic_ns()
        self.late_count = 0
        self.total_count = 0

    def sleep(self) -> float:
        """Dorme até o próximo tick. Devolve o atraso em ms (0 se em dia)."""
        self.total_count += 1
        now = time.monotonic_ns()
        if now < self._next_ns:
            time.sleep((self._next_ns - now) / 1e9)
            self._next_ns += self.period_ns
            return 0.0

        # Estouramos o prazo. Realinha ao próximo tick futuro em vez de tentar
        # "recuperar" o atraso com iterações em rajada.
        late_ns = now - self._next_ns
        self.late_count += 1
        missed = late_ns // self.period_ns + 1
        self._next_ns += missed * self.period_ns
        return late_ns / 1e6

    def reset(self) -> None:
        self._next_ns = time.monotonic_ns()

    @property
    def late_ratio(self) -> float:
        return self.late_count / self.total_count if self.total_count else 0.0


@dataclass
class LoopMonitor:
    """Estatísticas do período do loop, para diagnosticar engasgos.

    Guarda os últimos ``window`` períodos e reporta média, pior caso e
    percentil 95 — a média sozinha esconde travadas ocasionais, que são
    justamente o que estraga a coleta.
    """

    window: int = 200
    _samples: list[float] = field(default_factory=list)
    _last_ns: int = 0

    def tick(self) -> None:
        now = time.monotonic_ns()
        if self._last_ns:
            self._samples.append((now - self._last_ns) / 1e6)
            if len(self._samples) > self.window:
                del self._samples[: len(self._samples) - self.window]
        self._last_ns = now

    @property
    def hz(self) -> float:
        mean = self.mean_ms
        return 1000.0 / mean if mean > 0 else 0.0

    @property
    def mean_ms(self) -> float:
        return sum(self._samples) / len(self._samples) if self._samples else 0.0

    @property
    def max_ms(self) -> float:
        return max(self._samples) if self._samples else 0.0

    @property
    def p95_ms(self) -> float:
        if not self._samples:
            return 0.0
        ordered = sorted(self._samples)
        return ordered[min(int(len(ordered) * 0.95), len(ordered) - 1)]

    def summary(self) -> str:
        return (
            f"{self.hz:5.1f} Hz  média={self.mean_ms:5.1f} ms  "
            f"p95={self.p95_ms:5.1f} ms  máx={self.max_ms:6.1f} ms"
        )


__all__ = ["LoopMonitor", "RateLimiter"]
