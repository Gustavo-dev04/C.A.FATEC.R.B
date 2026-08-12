"""Treinamento dos modelos do C.A.FATEC.R.B.

Separado de ``jetson/robocar`` de propósito: o carro não precisa de torch para
gravar dataset nem para rodar (a inferência em prova usa TensorRT). Manter os
dois pacotes independentes evita arrastar 2 GB de dependências para o Jetson.

O esquema do dataset é importado de ``jetson/robocar/capture/schema.py`` — uma
definição só, para os dois lados nunca divergirem.
"""

__version__ = "0.1.0"

__all__ = ["__version__"]
