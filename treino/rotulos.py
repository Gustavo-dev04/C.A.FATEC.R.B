"""Definições compartilhadas por gerador de dados, treino e avaliação.

Sem PyTorch aqui: o gerador de dados roda em qualquer PC.
"""

from __future__ import annotations

from vista_sim.env import KAPPA_AHEAD_M
from vista_sim.track import SIGN_CLASSES

MANOBRAS = ("frente", "esquerda", "direita")
"""Ordem do vetor one-hot da manobra que entra na rede de faixa."""

ALVOS = ("e_y", "e_psi", "kappa", "dist_cruzamento")
"""Saídas de regressão da rede de faixa (unidades SI)."""

ESCALA = {"e_y": 0.10, "e_psi": 0.20, "kappa": 2.0, "dist_cruzamento": 1.0}
"""Divisor de cada alvo no treino: deixa tudo na ordem de 1."""

DIST_MAX = 3.0
"""Distância ao cruzamento é saturada aqui (m). Mais longe que isso não muda nada."""

KAPPA_M = KAPPA_AHEAD_M

CLASSES_DETECCAO = (*SIGN_CLASSES, "semaforo_vermelho", "semaforo_amarelo", "semaforo_verde")
"""Classes do detector (YOLO): as 6 placas da Figura 8 + o semáforo em cada estado."""

COLUNAS = (
    "arquivo", "semente", "quadro", "t",
    "e_y", "e_psi", "kappa", "dist_cruzamento", "no_cruzamento", "manobra",
    "placa_proxima", "semaforo", "v", "omega", "rpm_e", "rpm_d",
    "ruido", "pitch_deg", "vista", "fim",
)


def dividir_sementes(sementes: list[int], val_frac: float) -> tuple[set[int], set[int]]:
    """Treino × validação POR EPISÓDIO, nunca por quadro.

    Quadros do mesmo episódio são quase iguais; misturá-los entre treino e
    validação mede memória, não generalização. As últimas sementes vão para a
    validação.
    """
    s = sorted(set(sementes))
    n_val = max(1, round(len(s) * val_frac)) if len(s) > 1 else 0
    return set(s[: len(s) - n_val]), set(s[len(s) - n_val:])
