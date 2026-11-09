# ADR 006 — Treino: redes pré-treinadas ajustadas com dados do Vista-Sim

**Data:** 02/11/2026 · **Estado:** aceito (aprovado pela equipe; treino no Colab)

## Contexto

A pista é padronizada, o carro ainda não existe e faltam 32 dias para a prova.
O simulador já produz imagens com rótulos exatos (posição na faixa, placas,
semáforo, cruzamento) de graça. A equipe tem 100 unidades de computação no
Colab. CLAUDE.md: **não treinar na Jetson**.

## Decisão

### O que é aprendido e o que não é

| Peça | Como | Por quê |
|---|---|---|
| Rede de **faixa** | ResNet18 (ImageNet) ajustada | imagem → `e_y`, `e_ψ`, κ à frente, distância ao cruzamento, "dentro?" |
| **Detector** | YOLO11n (COCO) ajustado | 6 placas + semáforo nos 3 estados |
| Decisão no cruzamento | **regra fixa** (`rules.py`) | placa → manobra é lógica, não percepção |
| Controle | **lei fixa** (`controle.py`) | a mesma do oráculo (ADR 003), testada em 40/40 rotas |

A rede de faixa recebe a **manobra** como entrada (one-hot): dentro do
cruzamento a trilha certa depende de virar ou seguir — sem isso a mesma
imagem teria três respostas (imitação condicional).

### Dados (`tools/gerar_dataset.py`)

- 1 episódio = 1 semente = rota, cidade, luz, carro, IMU e montagem da câmera
  sorteados (pitch ±2°, roll/yaw ±1°); pós-processamento sorteado.
- Ruído do oráculo em 60 % dos episódios (alvo lateral deslizando a 5 cm/s,
  até 3,5 cm para o meio-fio e 6 cm para a contramão): 40/40 rotas completas.
- **Vistas deslocadas**: cada quadro salvo também é renderizado com o carro
  até ±15 cm de lado e ±15° torto, com os rótulos recalculados
  (`env.rotulos_em`). Cobertura de `e_y`: ±2,5 cm → −9…+15 cm (p5–p95).
- Semáforo **desenhado** em estado sorteado depois da largada (o carro obedece
  ao real): o detector vê vermelho/amarelo de vários ângulos.
- Determinístico: semente + commit regeram o mesmo dataset.

### Treino e validação

- Divisão treino/validação **por episódio**, nunca por quadro.
- Aumento de dados só fotométrico; **sem espelhar** (troca mão de direção,
  sinal de `e_y`, e "virar à esquerda" vira "à direita" — `fliplr=0` no YOLO).
- A métrica que decide é **dirigir no simulador** (`treino/avaliacao.py`):
  rotas completas em sementes nunca vistas, comparadas ao teto (mesmo piloto
  com a verdade).

### Entrega

ONNX + JSON com o contrato de entrada/saída. O motor TensorRT é gerado **na
Jetson** (`trtexec --fp16`), porque é específico do aparelho.

## Consequências

- O notebook `notebooks/treino_vista.ipynb` faz tudo: clona, gera, treina,
  exporta, avalia e salva no Drive.
- O simulador "fast" não é fotorrealista: a rede vai precisar de **imagens
  reais** da pista caseira para ajuste fino (S4). A randomização e as redes
  pré-treinadas reduzem, não eliminam, essa diferença.
- Nesta etapa a avaliação usa manobra e semáforo da verdade; o detector entra
  no laço depois.
