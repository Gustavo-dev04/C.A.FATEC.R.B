"""Regras de decisão por placa — a MESMA lógica que o Vista vai usar.

Fica separado do simulador de propósito: o oráculo decide com estas funções, e
a máquina de estados do Vista deve importar este módulo em vez de reescrevê-lo.
Assim o simulador testa exatamente a regra que vai para a pista.

O regulamento (item 3.5.2) só diz que "a rota será definida no dia, por placas
de trânsito". O que fazer em casos que ele não cobre é decisão NOSSA, marcada
abaixo como ``POLÍTICA`` — e listada em docs/regulamento.md como pergunta para a
organização.
"""

from __future__ import annotations

MANEUVERS = ("frente", "esquerda", "direita")

#: placa de obrigação → manobra
OBLIGATION = {
    "seguir_em_frente": "frente",
    "virar_esquerda": "esquerda",
    "virar_direita": "direita",
}
#: placa de proibição → manobra proibida
PROHIBITION = {
    "proibido_seguir_em_frente": "frente",
    "proibido_virar_esquerda": "esquerda",
    "proibido_virar_direita": "direita",
}


class SignalError(ValueError):
    """Sinalização impossível ou ambígua para as saídas do cruzamento."""


def decide(sign: str | None, options: set[str]) -> str:
    """Escolhe a manobra num cruzamento.

    Args:
        sign: classe da placa da aproximação, ou ``None`` se não há placa.
        options: manobras fisicamente possíveis ali (``frente``/``esquerda``/``direita``).

    Raises:
        SignalError: placa contradiz o cruzamento, ou deixa mais de uma saída.
    """
    if not options:
        raise SignalError("cruzamento sem saídas")
    unknown = set(options) - set(MANEUVERS)
    if unknown:
        raise SignalError(f"manobras desconhecidas: {unknown}")

    if sign is None:
        # POLÍTICA: sem placa, segue em frente. É o que a rota de exemplo das
        # Figuras 9–11 implica: há cruzamentos na rota sem placa, e a rota
        # passa reto por eles.
        if "frente" in options:
            return "frente"
        raise SignalError(f"sem placa e sem 'frente' possível: {sorted(options)}")

    if sign in OBLIGATION:
        m = OBLIGATION[sign]
        if m not in options:
            raise SignalError(f"placa '{sign}' manda '{m}', mas só há {sorted(options)}")
        return m

    if sign in PROHIBITION:
        remaining = set(options) - {PROHIBITION[sign]}
        if len(remaining) == 1:
            return next(iter(remaining))
        # POLÍTICA: proibição que deixa duas saídas é ambígua. Não chutamos.
        # O gerador de cenários nunca cria esse caso; se aparecer na pista
        # real, é pergunta para a organização.
        raise SignalError(f"'{sign}' deixa {sorted(remaining)} — ambíguo")

    raise SignalError(f"classe de placa desconhecida: {sign!r}")


def signs_that_force(maneuver: str, options: set[str]) -> list[str]:
    """Todas as placas que levam, sem ambiguidade, à ``maneuver``.

    Usado pelo gerador de cenários para sortear a placa de cada cruzamento da
    rota.
    """
    out = []
    for sign in (*OBLIGATION, *PROHIBITION):
        try:
            if decide(sign, options) == maneuver:
                out.append(sign)
        except SignalError:
            pass
    return out
