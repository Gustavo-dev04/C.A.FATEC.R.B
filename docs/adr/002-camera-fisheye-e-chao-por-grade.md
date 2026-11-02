# ADR 002 — Câmera fisheye em NumPy e chão por grade pré-calculada

- **Data:** 02/11/2026
- **Status:** aceito

## Contexto

O renderizador `fast` precisa de ≥ 1.000 passos/s em 320×240 (CLAUDE.md §5A)
com a câmera fisheye real do carro. Calcular a projeção de cada pixel a cada
quadro custaria caro.

## Decisão

1. **O modelo Kannala-Brandt (o do `cv2.fisheye`) é implementado em NumPy**
   em `vista_sim/camera.py`. Um teste confere que o pixel bate com
   `cv2.fisheye.projectPoints` com erro < 1e-6 px — então a calibração real,
   quando gravada em `config/camera/intrinsics.yaml`, vale igual no simulador.
2. **O chão é renderizado por uma grade pré-calculada.** Como a altura e o
   pitch da câmera em relação ao chão são fixos, o ponto onde cada pixel toca o
   chão — no referencial do carro — não muda com a pose. Ele é calculado uma
   vez; por quadro só se gira e translada essa grade e se faz um `cv2.remap`
   na textura do mapa (2 mm/px). A distorção fisheye já está embutida na grade.
3. **As linhas acima do horizonte** (que é fixo em relação à câmera) ficam fora
   do remap: o céu é copiado pronto.
4. **Placas e semáforo** são quadriláteros verticais com todos os cantos
   projetados numa única chamada vetorizada por quadro.

## Números medidos (1 núcleo, CPU do container)

| Resolução | Só render | Ambiente completo |
|---|---|---|
| 320×240 | **1.092 passos/s** | — |
| 410×308 (processamento) | 624 passos/s | 275 passos/s (9,2× tempo real) |
| 1640×1232 (sensor) | 58 passos/s | — |

Antes das otimizações 2–4 o render em 320×240 fazia 395 passos/s: o custo
estava 70 % nos sprites (projeção por placa, lista refeita a cada quadro).

## Consequências

- O carro inclina 0,3° por causa do rodízio (REV03, detalhe B) e isso **não**
  está na grade — ~1 px de deslocamento do horizonte na resolução de
  processamento. Desprezível frente à incerteza da calibração.
- Quarteirões e canteiros são planos: sem paralaxe dos 5 cm de altura e sem
  oclusão de placas atrás deles. O renderizador `photo` (S3) resolve.
- Sprites usam homografia dos 4 cantos projetados: ignora a curvatura do
  fisheye **dentro** da placa. Desprezível para 15 cm.
- Na resolução cheia do sensor o render fica abaixo de 200 passos/s. Isso só
  importa para gerar dataset em resolução cheia, que é papel do `photo`.
