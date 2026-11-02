# ADR 001 — O mapa do simulador vem do regulamento, não do .obj do organizador

- **Data:** 02/11/2026
- **Status:** aceito

## Contexto

O CLAUDE.md §5A sugeria converter os `.obj` do simulador do organizador
(`github.com/JulioZanotto/robocar_bot`) em mapa vetorial. Medindo esses `.obj`
e comparando com o regulamento — incluindo as cotas das figuras — apareceram
divergências que mudam o que a câmera vê:

| Item | `.obj` do organizador | Regulamento |
|---|---|---|
| Placa | Ø300, centro a 550 mm | Ø150, base a 400 (centro 475) |
| Classes de placa | 3 | 6 |
| Linha central | amarela | branca tracejada |
| Ilha central | Ø0,57 m | Ø1,4 m (cotas "80") |
| Semáforo | lentes ~24 mm | lentes Ø110 |

O próprio README do organizador chama o simulador de "réplica ainda em
desenvolvimento".

## Decisão

O mapa (`maps/robocar_master.yaml`) é **gerado por código** a partir de
parâmetros do regulamento (`vista_sim/maps/build_master.py`). O `.obj` serve
só para conferir a topologia — e ela bate: anel externo, 4 quarteirões, cruz de
canteiros e ilha central.

Cada número do gerador é marcado como `regulamento`, `medido` ou `inferido`, e
`tests/test_map.py` trava os do regulamento.

## Alternativas

- **Usar os `.obj`:** mais rápido, mas treinaria o detector com placas do
  dobro do tamanho real e sem as 3 classes de proibição.
- **Corrigir os `.obj` à mão:** frágil, e não deixaria rastro de qual número
  veio de onde.

## Consequências

- Mudou o regulamento ou uma inferência? Muda-se um parâmetro e regenera.
- O gerador é a base do gerador procedural (variações de largura, raio,
  espessura de linha), que é o que impede o Vista de decorar um mapa só.
- Inferências sem confirmação (mão dupla nas vias internas, ilha Ø1,4, raio dos
  cantos, espessura das linhas, posição do box) estão listadas em
  `docs/regulamento.md` como perguntas para a organização.
