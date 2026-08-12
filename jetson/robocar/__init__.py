"""C.A.FATEC.R.B — software embarcado do carro autônomo (RoboCar Race 2026).

Subpacotes:

* :mod:`robocar.comms`    — protocolo e enlace serial com o ESP32
* :mod:`robocar.sensors`  — captura de imagem
* :mod:`robocar.capture`  — gravação de dataset
* :mod:`robocar.control`  — fontes de comando para pilotar
* :mod:`robocar.config`   — carregamento dos YAML de ``config/``

Nada aqui importa torch, OpenCV ou pyserial no topo: importar ``robocar`` numa
máquina "pelada" precisa funcionar, e funciona.
"""

__version__ = "0.1.0"

__all__ = ["__version__"]
