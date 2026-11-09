# Regulamento RoboCar Race 2026 — o que importa para o Vista

Fonte: `docs/regulamento/RoboCar_Race_2026_Regulamento_v2026-05-22.pdf`
(versão de 22/05/2026). **Lido inclusive nas figuras** — a extração de texto do
PDF não pega as cotas desenhadas, e é nelas que estão vários números críticos.

Em caso de divergência, vale o PDF. Cada número abaixo diz de onde saiu.

---

## Datas

| Evento | Data |
|---|---|
| Treinos na pista oficial | **04/12/2026**, 14h–22h (único dia na pista real) |
| Prova | **05/12/2026**, a partir das 9h |

## Carro (Júnior/Master, item 3.4.1)

- Máximo **500 × 250 mm**. Sem mínimo. O nosso: 400 × 241 mm (REV03).
- 2 ou 4 rodas tracionadas tocando o chão.
- **Sensoriamento 100 % embarcado. Nada de comunicação com computação ou
  sensoriamento externo.** Teleoperação só em treino/teste.
- "Permitido o uso de **câmeras**, GPS, Lidar e qualquer outro tipo de
  sensoriamento" — no plural. **Não há restrição de lente, FOV, resolução ou
  número de câmeras.** A única lista fechada de componentes é da categoria NANO.
- Item 4.9: proibido trocar o robô ou "troca substancial dos componentes
  embarcados" durante o torneio.

## Pista (Figuras 6 e 7, cotas em cm)

| Elemento | Valor | Fonte |
|---|---|---|
| Cidade | **13,0 × 9,0 m** | 3.4.1 |
| Quarteirões | **4,0 × 2,0 m** (4, nos cantos) | legenda da Fig. 7 |
| Anel externo | **100 cm** | Fig. 6, cota "100" |
| Vias norte-sul (ao lado do canteiro vertical) | **100 cm** | Fig. 6, "100 \| 100" |
| Vias leste-oeste (ao lado do canteiro horizontal) | **80 cm** | Fig. 6, "80" |
| Folga entre ponta do canteiro e a ilha central | **80 cm** | Fig. 6, "80 \| Fatec \| 80" |
| Piso | carpete **preto/cinza** com **delimitação branca** | 3.5.2 |
| Linha central | **tracejado branco** | vista 3D, Fig. 7 |

Derivado das cotas (o regulamento fecha sozinho): canteiro vertical de 1,0 m,
canteiro horizontal de 1,4 m, ilha central de Ø1,4 m.

- **Manter-se na mão direita.** "Cada invasão na contramão ou na saída da
  pista será penalizada com **−10,0 segundos**." ⚠️ O sinal está ambíguo: a
  classificação é por menor tempo, e −10 s seria prêmio. Assumimos +10 s.
- Largada/chegada no anel de cima, à direita (Figs. 6, 9, 10). A rota de
  exemplo sai para **oeste**.
- "No fim, parar sozinho no box." O box **não é descrito**.

## Placas — só MASTER (item 3.5.2 + Figura 8)

| Parâmetro | Valor |
|---|---|
| Diâmetro | **150 mm** |
| Altura do chão até o ponto **INFERIOR** | **400 mm** → centro a **475 mm**, topo a 550 mm |
| Posição | sempre **400 mm antes** do cruzamento, **à direita**, tangenciando a calçada |

**São 6 classes, não 3** (Figura 8, detalhe):

| Obrigação | Proibição |
|---|---|
| seguir em frente | proibido seguir em frente |
| virar à esquerda | proibido virar à esquerda |
| virar à direita | proibido virar à direita |

As de proibição **invertem a lógica de decisão**: "proibido seguir em frente"
num cruzamento obriga a virar, e o lado não vem da placa. A máquina de estados
precisa tratar isso — não é só uma questão de `signs.yaml`.

Na Figura 9, a placa do anel de cima está **fora da pista**, depois da borda
externa — porque ali a direita do carro é a borda, não um quarteirão.

Vídeo oficial das placas: <https://youtu.be/YAg2p1kShZA>

**Consequência para a câmera (medido no Vista-Sim, 09/11):** com a câmera a
15,6 cm do chão e 15° para baixo (meio-FOV vertical ≈ 36°), o centro da placa
(475 mm) sai pelo topo da imagem quando ela está a menos de ~0,8 m. Nas ruas
curtas do centro (faixa de ~1 m) a placa do cruzamento seguinte quase nunca
aparece na própria aproximação — só de longe. Por isso o Vista acumula as
leituras de TODAS as placas que vê, cada uma atribuída à sua aproximação pelo
mapa (ADR 008). Em 24 rotas (233 placas de rota, sem contar prédios na frente):

| Pitch | Legível (≥ 4 leituras) em algum momento | … na própria aproximação | Nunca |
|---|---|---|---|
| 15° (atual) | 98,7 % | 66,5 % | 1,3 % |
| 10° | 98,7 % | 73,8 % | 0,9 % |
| 7° | 99,1 % | 83,7 % | 0,9 % |
| 5° | 99,1 % | 85,0 % | 0,9 % |
| 0° | 99,1 % | 85,4 % | 0,0 % |

## Semáforo (item 3.6 + figura)

| Parâmetro | Valor |
|---|---|
| Distância do ponto de largada | **100 cm** |
| Caixa | **140 × 470 mm** |
| Lentes | **Ø110 mm**, **140 mm** entre centros |
| Cota "390" | do chão ao **CENTRO da lente AMARELA** |
| Centros das lentes | vermelho **530**, amarelo **390**, verde **250** mm |
| Sequência | vermelho → amarelo → verde, uma cor por vez |

A leitura da cota 390 foi **medida por pixel** na figura renderizada a 400 dpi:
espaçamentos de 356,5 e 348,5 px para 140 mm, e a cota termina na linha do
centro da lente do meio. A caixa medida dá 609 × 152 mm contra 470 × 140
cotados — erro de traço do desenho, mas a estrutura é inequívoca.

Furar o verde: **+20 s** se completar, **−2 m** se não completar.

Vídeo oficial do semáforo: <https://www.youtube.com/watch?v=GvytKrULBW0>

## Divergências com o simulador do organizador

`github.com/JulioZanotto/robocar_bot` — o próprio README o chama de "réplica
ainda em desenvolvimento". Medido nos `.obj`:

| Item | Simulador do organizador | Regulamento |
|---|---|---|
| Placa | Ø300, centro a 550 mm | **Ø150, base a 400 (centro 475)** |
| Classes de placa | 3 | **6** |
| Linha central | amarela | **branca tracejada** |
| Ilha central | Ø0,57 m | **Ø1,4 m** (derivado das cotas "80") |
| Semáforo | lentes ~24 mm, ciclo verm. → verde → amarelo | **Ø110, verm. → amarelo → verde** |

O Vista-Sim usa o **regulamento**. Ver `docs/adr/001-mapa-pelo-regulamento.md`.

## Perguntas para a organização

**Decisão da equipe (09/11): as perguntas NÃO serão enviadas.** Ficam aqui
como registro do que o regulamento não responde; onde o Vista precisa de uma
resposta, vale a política anotada (e a confirmação na pista, 04/12).

Endereço, caso mudem de ideia: robocar.race@gmail.com.

1. A penalidade por invasão/saída é **+10 s**? O texto diz "−10,0 segundos".
2. As vias internas são de **mão dupla** (2 faixas de 40/50 cm)? A rota da
   Figura 10/11 sugere que sim.
3. Diâmetro da ilha central (as cotas implicam Ø1,4 m; o simulador tem Ø0,57).
4. Como é o **box** de chegada e como se reconhece que o carro chegou?
5. As 6 placas da Figura 8 são a lista **fechada**?
6. Raio dos cantos externos da pista (não cotado).
7. Espessura das linhas pintadas (não cotada).
8. Existem **faixas de pedestre** na pista? O regulamento não fala delas. O
   Vista-Sim as sorteia (zebra 1:10 nas bocas de rua) só para o Vista não
   confundir zebra com faixa — se existirem, pedir posição e medidas.
9. **Cruzamento sem placa**: o carro segue em frente? (É o que a rota das
   Figuras 9–11 sugere.) → **Política confirmada pela equipe em 09/11**:
   sem placa = em frente (`vista_sim/rules.py`).
10. Uma placa de **proibição** pode deixar duas saídas possíveis (ex.:
    "proibido virar à esquerda" num cruzamento de 4 vias)? Se puder, qual
    saída vale? → **Sem resposta.** O gerador de cenários nunca cria esse
    caso; se o Vista ler uma, usa a **reserva provisória** frente → direita →
    esquerda (a que a placa não proíbe) e registra `alternativa=True`
    (`vista/placas.py`). A decidir pela equipe.
11. Como **termina** a prova da MASTER — volta ao ponto de largada? O "box" só
    aparece descrito nas seções NANO/MICRO/MINI.
12. Haverá **maquetes de prédios** nos quarteirões (3.4.1 permite)? Altura
    máxima? Elas escondem placas e fazem sombra na pista.
