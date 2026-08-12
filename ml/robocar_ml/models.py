"""Modelos de condução.

Começamos com a PilotNet (NVIDIA, "End to End Learning for Self-Driving Cars",
2016) porque ela é o baseline honesto deste problema: ~250 mil parâmetros,
roda folgado no Orin Nano e resolve manutenção de faixa com poucas horas de
dado. Trocar por algo maior antes de ter dado suficiente é otimizar a parte
errada.
"""

from __future__ import annotations

import torch
import torch.nn as nn


class PilotNet(nn.Module):
    """CNN de regressão para direção, entrada 3x66x200.

    Args:
        outputs: 1 = só esterço; 2 = esterço e aceleração.
        dropout: regularização nas camadas densas. Com dataset pequeno
            (poucas sessões), subir para 0.5 ajuda.
    """

    def __init__(self, outputs: int = 1, dropout: float = 0.3) -> None:
        super().__init__()

        self.features = nn.Sequential(
            # Convoluções com passo 2: reduzem rápido a resolução e olham
            # estrutura de faixa, não textura fina do carpete.
            nn.Conv2d(3, 24, kernel_size=5, stride=2),
            nn.BatchNorm2d(24),
            nn.ELU(),
            nn.Conv2d(24, 36, kernel_size=5, stride=2),
            nn.BatchNorm2d(36),
            nn.ELU(),
            nn.Conv2d(36, 48, kernel_size=5, stride=2),
            nn.BatchNorm2d(48),
            nn.ELU(),
            nn.Conv2d(48, 64, kernel_size=3),
            nn.BatchNorm2d(64),
            nn.ELU(),
            nn.Conv2d(64, 64, kernel_size=3),
            nn.BatchNorm2d(64),
            nn.ELU(),
        )

        self.regressor = nn.Sequential(
            nn.Flatten(),
            nn.Dropout(dropout),
            nn.Linear(64 * 1 * 18, 100),
            nn.ELU(),
            nn.Dropout(dropout),
            nn.Linear(100, 50),
            nn.ELU(),
            nn.Linear(50, 10),
            nn.ELU(),
            nn.Linear(10, outputs),
            # tanh mantém a saída em [-1, 1], exatamente a faixa que o
            # protocolo aceita. Sem isso, um modelo mal treinado pediria
            # esterço 3.7 e o clamp esconderia o problema.
            nn.Tanh(),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.regressor(self.features(x))


def build_model(name: str = "pilotnet", **kwargs) -> nn.Module:
    models = {"pilotnet": PilotNet}
    if name not in models:
        raise ValueError(f"modelo desconhecido: {name!r} (disponíveis: {list(models)})")
    return models[name](**kwargs)


def count_parameters(model: nn.Module) -> int:
    return sum(p.numel() for p in model.parameters() if p.requires_grad)


__all__ = ["PilotNet", "build_model", "count_parameters"]
