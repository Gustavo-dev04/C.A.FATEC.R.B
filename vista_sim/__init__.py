"""Vista-Sim — simulador do C.A.FATEC.R.B para a RoboCar Race 2026 (MASTER).

Feito só para este carro e esta prova (CLAUDE.md §5A). Dois usos:
1. gerar dados de treino rotulados automaticamente;
2. testar o Vista em loop fechado, mais rápido que o tempo real.

Fonte única de verdade: lê ``config/car.yaml`` e ``config/camera/*.yaml``,
os mesmos arquivos do carro real.
"""

__version__ = "0.1.0"
