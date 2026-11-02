# ADR 004 — Cenários sorteados: rota pela cidade, placas pela regra, luz com sombra

**Data:** 02/11/2026 · **Estado:** aceito

## Contexto

O primeiro vídeo do simulador era uma volta no anel externo: não entrava na
cidade, as placas eram fixas e não mandavam em nada, não havia faixa de
pedestre e a luz era uniforme. Na prova, a rota é definida **no dia, por
placas** (3.5.2), e a luz de um ginásio vem de lâmpadas discretas — mesmo
branca, o que fica entre a lâmpada e o chão faz sombra.

## Decisão

### Malha viária (`vista_sim/roads.py`)

- 12 cruzamentos (8 T no anel, 4 em volta da ilha) e 20 ruas, todas de mão
  dupla; a faixa é a metade direita de cada rua.
- A posição de **cada placa** sai da regra 3.5.2 aplicada à malha — 400 mm
  antes do cruzamento, medidos ao longo da rua, à direita, na calçada.
  Calculada, não escrita à mão: vale para qualquer rota.

### Regras de decisão (`vista_sim/rules.py`)

- Obrigação → manobra. Proibição → vale só se sobra **uma** saída.
- **Política nossa** (o regulamento não diz): sem placa, segue em frente;
  proibição que deixa duas saídas é erro (não chutamos).
- O oráculo decide com estas funções, e a máquina de estados do Vista deve
  **importar** este módulo — o simulador testa a regra que vai para a pista.

### Cenário (`vista_sim/scenario.py`), por semente

- **Rota**: passeio aleatório pela cidade e volta à largada; uma aproximação
  usada duas vezes pede sempre a mesma manobra. 25–66 m, 6–15 cruzamentos;
  99 % das rotas entram no centro (200 sementes).
- **Placas**: para cada cruzamento da rota, uma placa que leva à manobra pela
  regra acima (o gerador **confere** com `decide`). As 6 classes aparecem.
  Placas **distratoras** em aproximações fora da rota.
- **Faixas de pedestre**: zebras nas bocas de rua, escala 1:10. O regulamento
  não fala delas — ver pergunta em `docs/regulamento.md`.
- **Maquetes** nos quarteirões (3.4.1 permite), público e juízes em volta.
- **Luz**: 2–8 lâmpadas a 3–9 m, queda cos³, tom de 3500–6500 K, luz
  ambiente, exposição travada (a câmera não compensa).

### Iluminação (`vista_sim/lighting.py`)

- Sombra de cada obstáculo (prédio, pessoa, placa, semáforo) projetada de cada
  lâmpada, com penumbra. O mapa de luz (2 cm/px) é **assado na textura do
  chão** uma vez por cenário — custo zero por quadro.
- A **sombra do próprio carro** anda com ele: desenhada no referencial do
  carro e levada à imagem por um remap fixo; recalculada a cada 4 cm / 2°.
- Rótulo `visible_frac` em cada placa: quanto dela não está escondida atrás de
  prédio ou pessoa.

### Fiscal da rota (`vista_sim/env.py`)

- Verdade de campo medida numa janela ao longo da rota (a rota passa duas
  vezes pelo mesmo cruzamento).
- Eventos: `placa_respeitada`, `desrespeitou_placa` (com esperado × feito),
  `saiu_da_pista`, `saiu_da_rota`, `entrou_na_contramao`,
  `largou_antes_do_verde`, `rota_completa`.

## Consequências

- O oráculo completa **200 de 200** rotas sorteadas sem desrespeitar placa nem
  sair da pista (erro lateral máx. < 3 cm).
- Custo: ~240 passos/s com a cidade inteira (prédios, público, placas, luz) em
  320×240 contra ~840 só com o chão; `env.step` completo a 204 passos/s em
  410×308. Para gerar dados, roda-se 1 processo por núcleo.
- Faixa de pedestre, maquetes e luz são **sorteadas** até a organização
  responder — servem para o Vista não depender delas, não para imitá-las.
