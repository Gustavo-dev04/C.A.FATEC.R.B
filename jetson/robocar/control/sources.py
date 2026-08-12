"""Fontes de comando para pilotar o carro durante a coleta de dados.

Três opções, em ordem de qualidade do dado gerado:

1. :class:`RcSource` — o piloto usa o rádio RC e o ESP32 aplica direto. O
   Jetson não comanda nada, só grava. **É a melhor opção**: entrada analógica,
   latência mínima e o piloto olha para o carro, não para a tela.
2. :class:`GamepadSource` — analógico, mas passa pelo Jetson. Boa alternativa
   quando não há rádio.
3. :class:`KeyboardSource` — só a biblioteca padrão, mas discreto (liga/desliga).
   Serve para teste de bancada; **evite coletar dataset com ele** — degraus de
   esterço ensinam a rede a virar em degraus.

Regulamento: qualquer uma destas é permitida apenas em treino e teste. Na volta
oficial, nada de controle externo (ver ``config/profiles/race.yaml``).
"""

from __future__ import annotations

import contextlib
import logging
import sys
import termios
import time
import tty
from abc import ABC, abstractmethod
from select import select
from typing import Any

log = logging.getLogger(__name__)


class ControlSource(ABC):
    """Produz ``(steer, throttle)`` normalizados em [-1, 1]."""

    name = "base"

    @abstractmethod
    def read(self) -> tuple[float, float]: ...

    def close(self) -> None:
        return None

    def __enter__(self) -> ControlSource:
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()


class RcSource(ControlSource):
    """O rádio RC comanda; o Jetson não envia comando de atuação.

    Ainda assim é preciso enviar ``$CMD`` periodicamente, senão o failsafe do
    ESP32 dispara. Enviamos zeros: com ``rc_enable=1`` o firmware dá prioridade
    ao rádio e usa o ``$CMD`` apenas como batimento cardíaco.
    """

    name = "rc"

    def read(self) -> tuple[float, float]:
        return 0.0, 0.0


class GamepadSource(ControlSource):
    """Controle analógico via ``evdev`` (Linux).

    Testado com Xbox 360/One e DualShock. O mapeamento de eixos varia por
    modelo — rode ``robocar teleop --list-axes`` para descobrir o do seu.
    """

    name = "gamepad"

    def __init__(
        self,
        device: str | None = None,
        steer_axis: str = "ABS_X",
        throttle_axis: str = "ABS_RZ",
        brake_axis: str = "ABS_Z",
        deadzone: float = 0.08,
        expo: float = 0.5,
    ) -> None:
        try:
            import evdev  # type: ignore[import-not-found]
        except ImportError as exc:  # pragma: no cover - ambiente sem evdev
            raise RuntimeError(
                "evdev não instalado. Rode: pip install evdev"
            ) from exc
        self._evdev = evdev

        path = device or self._autodetect()
        self._device = evdev.InputDevice(path)
        self._device.grab()
        log.info("gamepad: %s (%s)", self._device.name, path)

        self._steer_axis = steer_axis
        self._throttle_axis = throttle_axis
        self._brake_axis = brake_axis
        self._deadzone = deadzone
        self._expo = expo

        self._ranges = {}
        for code, absinfo in self._device.capabilities().get(evdev.ecodes.EV_ABS, []):
            self._ranges[evdev.ecodes.ABS[code]] = (absinfo.min, absinfo.max)

        self._raw = {steer_axis: 0.0, throttle_axis: 0.0, brake_axis: 0.0}

    def _autodetect(self) -> str:
        import evdev  # type: ignore[import-not-found]

        for path in evdev.list_devices():
            device = evdev.InputDevice(path)
            caps = device.capabilities()
            if evdev.ecodes.EV_ABS in caps and evdev.ecodes.EV_KEY in caps:
                return path
        raise RuntimeError(
            "nenhum gamepad encontrado. Conecte o controle e confira `ls /dev/input/event*`"
        )

    def _normalize(self, axis: str, value: float, bipolar: bool) -> float:
        low, high = self._ranges.get(axis, (0, 255))
        span = high - low
        if span == 0:
            return 0.0
        scaled = (value - low) / span
        return scaled * 2.0 - 1.0 if bipolar else scaled

    def read(self) -> tuple[float, float]:
        # Drena todos os eventos pendentes: interessa o estado mais recente,
        # não o histórico.
        while True:
            event = self._device.read_one()
            if event is None:
                break
            if event.type == self._evdev.ecodes.EV_ABS:
                name = self._evdev.ecodes.ABS.get(event.code)
                if isinstance(name, list):
                    name = name[0]
                if name in self._raw:
                    self._raw[name] = event.value

        steer = self._normalize(self._steer_axis, self._raw[self._steer_axis], True)
        forward = self._normalize(self._throttle_axis, self._raw[self._throttle_axis], False)
        reverse = self._normalize(self._brake_axis, self._raw[self._brake_axis], False)

        steer = apply_deadzone(steer, self._deadzone)
        steer = apply_expo(steer, self._expo)
        throttle = forward - reverse
        return steer, max(-1.0, min(1.0, throttle))

    def close(self) -> None:
        try:
            self._device.ungrab()
            self._device.close()
        except Exception:  # pragma: no cover
            pass


class KeyboardSource(ControlSource):
    """Teclado em modo raw (só biblioteca padrão).

    Teclas: ``a``/``d`` esterço, ``w``/``s`` aceleração, ``espaço`` para
    tudo, ``q`` sai. Sem tecla pressionada, os valores decaem para zero — como
    num controle com mola de retorno.
    """

    name = "keyboard"

    def __init__(
        self,
        steer_step: float = 0.12,
        throttle_step: float = 0.06,
        decay_per_s: float = 2.5,
    ) -> None:
        if not sys.stdin.isatty():
            raise RuntimeError("KeyboardSource exige um terminal interativo (TTY)")
        self._steer_step = steer_step
        self._throttle_step = throttle_step
        self._decay = decay_per_s
        self._steer = 0.0
        self._throttle = 0.0
        self._last_ns = time.monotonic_ns()
        self.quit_requested = False

        self._fd = sys.stdin.fileno()
        self._saved = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)

    def read(self) -> tuple[float, float]:
        now = time.monotonic_ns()
        dt = (now - self._last_ns) / 1e9
        self._last_ns = now

        pressed = set()
        while select([sys.stdin], [], [], 0)[0]:
            char = sys.stdin.read(1)
            if not char:
                break
            pressed.add(char.lower())

        if "q" in pressed:
            self.quit_requested = True
        if " " in pressed:
            self._steer = self._throttle = 0.0
            return 0.0, 0.0

        if "a" in pressed:
            self._steer -= self._steer_step
        if "d" in pressed:
            self._steer += self._steer_step
        if "w" in pressed:
            self._throttle += self._throttle_step
        if "s" in pressed:
            self._throttle -= self._throttle_step

        # Sem tecla no eixo, volta ao centro.
        if not pressed & {"a", "d"}:
            self._steer = _decay_to_zero(self._steer, self._decay * dt)
        if not pressed & {"w", "s"}:
            self._throttle = _decay_to_zero(self._throttle, self._decay * dt)

        self._steer = max(-1.0, min(1.0, self._steer))
        self._throttle = max(-1.0, min(1.0, self._throttle))
        return self._steer, self._throttle

    def close(self) -> None:
        # Restaurar o terminal é obrigatório: sem isso, o shell fica sem eco
        # depois de um Ctrl+C e o usuário precisa rodar `reset`.
        with contextlib.suppress(Exception):  # pragma: no cover
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)


def _decay_to_zero(value: float, amount: float) -> float:
    if value > 0:
        return max(0.0, value - amount)
    return min(0.0, value + amount)


def apply_deadzone(value: float, deadzone: float) -> float:
    """Zera a região central e reescala o resto, evitando degrau na saída."""
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def apply_expo(value: float, expo: float) -> float:
    """Curva exponencial: mais resolução perto do centro.

    ``expo=0`` é linear; ``expo=1`` é cúbico puro. Em carro 1:10, alguma expo
    no esterço melhora muito a qualidade do dado — o piloto consegue fazer
    correções finas em reta sem perder o esterço total nas curvas.
    """
    expo = max(0.0, min(1.0, expo))
    return (1.0 - expo) * value + expo * value**3


def make_source(kind: str, **kwargs: Any) -> ControlSource:
    """Fábrica usada pela CLI (``--control rc|gamepad|keyboard``)."""
    kinds = {
        "rc": RcSource,
        "gamepad": GamepadSource,
        "keyboard": KeyboardSource,
    }
    if kind not in kinds:
        raise ValueError(
            f"fonte de controle desconhecida: {kind!r} (use: {', '.join(kinds)})"
        )
    return kinds[kind](**kwargs)  # type: ignore[abstract]


__all__ = [
    "ControlSource",
    "GamepadSource",
    "KeyboardSource",
    "RcSource",
    "apply_deadzone",
    "apply_expo",
    "make_source",
]
