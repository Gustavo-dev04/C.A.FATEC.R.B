"""Treino do Vista com dados do Vista-Sim.

* ``rotulos``   — definições comuns (classes, alvos, divisão por episódio);
* ``faixa``     — rede de faixa (PyTorch): imagem + manobra → e_y, e_ψ, κ, cruzamento;
* ``deteccao``  — monta o dataset do detector de placas/semáforo (YOLO);
* ``avaliacao`` — o modelo dirigindo no simulador (a métrica que importa).

Treino roda no PC ou no Colab — **nunca na Jetson** (CLAUDE.md).
"""
