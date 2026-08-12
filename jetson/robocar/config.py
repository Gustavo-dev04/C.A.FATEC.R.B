"""Carregamento e mesclagem dos arquivos de configuração.

Os YAML em ``config/`` são a fonte única de verdade. Nada de constante de
calibração espalhada pelo código: se um número descreve o carro físico, ele
mora em ``config/vehicle.yaml``.
"""

from __future__ import annotations

import copy
import os
from pathlib import Path
from typing import Any, Iterable

import yaml

CONFIG_FILES = ("vehicle.yaml", "camera.yaml", "capture.yaml")

_MISSING = object()
"""Sentinela para distinguir "sem default" de ``default=None``."""


class ConfigError(RuntimeError):
    """Configuração ausente, malformada ou incoerente."""


def find_repo_root(start: Path | None = None) -> Path:
    """Sobe a árvore procurando a raiz do repositório.

    Permite rodar ``robocar`` de qualquer diretório sem passar caminho.
    """
    if env := os.environ.get("ROBOCAR_ROOT"):
        return Path(env).expanduser().resolve()

    current = (start or Path(__file__)).resolve()
    for candidate in (current, *current.parents):
        if (candidate / "config" / "vehicle.yaml").exists():
            return candidate
    raise ConfigError(
        "raiz do repositório não encontrada (procurando config/vehicle.yaml). "
        "Defina ROBOCAR_ROOT ou rode de dentro do repositório."
    )


def deep_merge(base: dict, override: dict) -> dict:
    """Mescla ``override`` sobre ``base`` recursivamente, sem mutar nenhum dos dois."""
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


class Config:
    """Acesso por caminho pontilhado a um dicionário aninhado.

    ``cfg["steering.center_us"]`` em vez de
    ``cfg["steering"]["center_us"]`` — e com erro legível quando falta chave.
    """

    def __init__(self, data: dict[str, Any], source: str = "<memória>") -> None:
        self._data = data
        self.source = source

    def __repr__(self) -> str:
        return f"Config(source={self.source!r}, keys={sorted(self._data)})"

    @property
    def data(self) -> dict[str, Any]:
        return self._data

    def get(self, path: str, default: Any = _MISSING) -> Any:
        node: Any = self._data
        for part in path.split("."):
            if not isinstance(node, dict) or part not in node:
                if default is _MISSING:
                    raise ConfigError(f"chave ausente em {self.source}: '{path}'")
                return default
            node = node[part]
        return node

    def __getitem__(self, path: str) -> Any:
        return self.get(path)

    def __contains__(self, path: str) -> bool:
        return self.get(path, None) is not None

    def section(self, path: str) -> Config:
        value = self.get(path)
        if not isinstance(value, dict):
            raise ConfigError(f"'{path}' não é uma seção em {self.source}")
        return Config(value, source=f"{self.source}:{path}")

    def as_dict(self) -> dict[str, Any]:
        return copy.deepcopy(self._data)

    # Acesso tipado: falha alto e claro em vez de propagar tipo errado.

    def get_int(self, path: str, default: Any = _MISSING) -> int:
        return int(self.get(path, default))

    def get_float(self, path: str, default: Any = _MISSING) -> float:
        return float(self.get(path, default))

    def get_bool(self, path: str, default: Any = _MISSING) -> bool:
        value = self.get(path, default)
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "sim", "on"}
        return bool(value)

    def get_str(self, path: str, default: Any = _MISSING) -> str:
        return str(self.get(path, default))


def load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise ConfigError(f"arquivo de configuração não encontrado: {path}")
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ConfigError(f"YAML inválido em {path}: {exc}") from exc
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ConfigError(f"{path} deve conter um mapeamento no topo")
    return data


def load_config(
    root: Path | None = None,
    profile: str | None = None,
    *,
    files: Iterable[str] = CONFIG_FILES,
) -> Config:
    """Carrega e mescla toda a configuração do projeto.

    Args:
        root: raiz do repositório. Descoberta automaticamente se omitida.
        profile: nome de um arquivo em ``config/profiles/`` (ex.: ``race``),
            cujo conteúdo é mesclado por cima de tudo.

    Returns:
        Um :class:`Config` com as seções ``vehicle``, ``steering``,
        ``throttle``, ``safety``, ``camera``, ``capture`` etc. no nível raiz.
    """
    root = Path(root) if root else find_repo_root()
    config_dir = root / "config"

    merged: dict[str, Any] = {}
    for name in files:
        merged = deep_merge(merged, load_yaml(config_dir / name))

    if profile:
        profile_path = config_dir / "profiles" / f"{profile}.yaml"
        merged = deep_merge(merged, load_yaml(profile_path))

    config = Config(merged, source=str(config_dir))
    validate(config)
    return config


def validate(config: Config) -> None:
    """Checa incoerências que só apareceriam como comportamento estranho.

    Barato de rodar na largada, caríssimo de descobrir na pista.
    """
    problems: list[str] = []

    center = config.get_int("steering.center_us", 1500)
    smin = config.get_int("steering.min_us", 1000)
    smax = config.get_int("steering.max_us", 2000)
    if not smin < center < smax:
        problems.append(
            f"steering: center_us ({center}) precisa estar entre min_us ({smin}) e max_us ({smax})"
        )
    if not (smin >= 800 and smax <= 2400):
        problems.append(
            f"steering: pulsos fora da faixa segura de servo (800-2400 us): {smin}-{smax}"
        )

    neutral = config.get_int("throttle.neutral_us", 1500)
    tmin = config.get_int("throttle.min_us", 1000)
    tmax = config.get_int("throttle.max_us", 2000)
    if not tmin <= neutral <= tmax:
        problems.append(
            f"throttle: neutral_us ({neutral}) fora de [{tmin}, {tmax}]"
        )

    for key in ("throttle.limit_collect", "throttle.limit_auto", "throttle.limit_race"):
        value = config.get_float(key, 0.0)
        if not 0.0 <= value <= 1.0:
            problems.append(f"{key} fora de [0, 1]: {value}")

    timeout = config.get_int("safety.command_timeout_ms", 250)
    command_hz = config.get_int("safety.command_hz", 50)
    if timeout < 3 * 1000 / max(command_hz, 1):
        problems.append(
            f"safety: command_timeout_ms ({timeout}) é curto demais para "
            f"command_hz ({command_hz}) — o failsafe vai disparar sozinho. "
            f"Use pelo menos {int(3 * 1000 / command_hz)} ms."
        )

    stop_mm = config.get_int("safety.obstacle_stop_mm", 0)
    slow_mm = config.get_int("safety.obstacle_slow_mm", 0)
    if stop_mm >= slow_mm:
        problems.append(
            f"safety: obstacle_stop_mm ({stop_mm}) deve ser menor que "
            f"obstacle_slow_mm ({slow_mm})"
        )

    # Limites do regulamento para Júnior/Master (item 3.4.1).
    length = config.get_int("vehicle.length_mm", 0)
    width = config.get_int("vehicle.width_mm", 0)
    if length > 500:
        problems.append(f"vehicle.length_mm ({length}) excede o limite de 500 mm")
    if width > 250:
        problems.append(f"vehicle.width_mm ({width}) excede o limite de 250 mm")

    rate = config.get_int("capture.rate_hz", 20)
    fps = config.get_int("camera.capture.fps", 30)
    if rate > fps:
        problems.append(
            f"capture.rate_hz ({rate}) é maior que camera.capture.fps ({fps}) — "
            "a gravação nunca alcançaria essa taxa"
        )

    if problems:
        raise ConfigError(
            "configuração inconsistente:\n  - " + "\n  - ".join(problems)
        )


def throttle_limit_for(config: Config, mode: str) -> float:
    """Teto de aceleração do modo de operação (``collect``/``auto``/``race``)."""
    key = {
        "collect": "throttle.limit_collect",
        "auto": "throttle.limit_auto",
        "race": "throttle.limit_race",
    }.get(mode)
    if key is None:
        raise ConfigError(f"modo desconhecido: {mode!r}")
    # Um perfil pode apertar o teto (nunca afrouxar) via `throttle_limit`.
    limit = config.get_float(key)
    override = config.get_float("throttle_limit", limit)
    return min(limit, override)


__all__ = [
    "CONFIG_FILES",
    "Config",
    "ConfigError",
    "deep_merge",
    "find_repo_root",
    "load_config",
    "load_yaml",
    "throttle_limit_for",
    "validate",
]
