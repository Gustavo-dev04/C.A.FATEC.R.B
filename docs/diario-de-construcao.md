# Diário de construção — C.A.FATEC.R.B

Registro de todo o caminho do projeto: o que foi feito, os problemas, as causas
e como foram resolvidos, com números medidos. **Só se acrescenta; entradas
antigas nunca são reescritas.** (CLAUDE.md §9, regra 10.)

> ⚠️ As entradas da **parte física até 02/11/2026** estão na versão inicial do
> Guga, que ainda não está neste repositório. Elas devem ser coladas **acima**
> desta linha, na ordem cronológica, antes da entrada de 02/11 abaixo.

---

## 02/11/2026 — Vista-Sim S1: mapa oficial, carro, câmera fisheye e volta completa

### Feito

- `config/car.yaml` com a geometria do desenho **RC-2026-REV03** (folhas M1–M3):
  bitola 214 mm, roda Ø65, eixo a 60 mm da traseira da P01, rodízio a 280 mm do
  eixo, P01 a 66,5 mm do chão, ultrassônicos S1–S4, IMU.
- `config/camera/intrinsics.yaml` e `extrinsics.yaml`, provisórios até a
  calibração.
- `vista_sim/`: gerador do mapa a partir do regulamento, cinemática diferencial
  com dinâmica de 1ª ordem por roda e encoder quantizado, câmera fisheye
  (Kannala-Brandt), renderizador `fast`, ambiente no estilo `gymnasium`,
  piloto-oráculo (pure pursuit).
- `tools/sim_demo.py`: uma volta no anel externo com vídeo e benchmark.
- 41 testes.

### Números

| Medida | Valor |
|---|---|
| Render 320×240 | **1.092 passos/s** (meta S1: ≥ 1.000) ✅ |
| Render 410×308 | 624 passos/s |
| Ambiente completo 410×308 | 275 passos/s = **9,2× tempo real** |
| Volta do oráculo no anel externo (0,5 m/s) | 83,3 s, 40,7 m de faixa |
| `e_y` do oráculo | RMS 0,12 cm, máx. 0,52 cm |
| Largada antes do verde | 0 |

### Problemas, causas e soluções

**1. O alerta geométrico de prioridade 1 estava baseado numa leitura errada da
figura do semáforo.**
- *Sintoma:* o CLAUDE.md concluía que com pitch de 15° a lente vermelha ficava
  fora da imagem, empurrando o pitch para 5°.
- *Causa:* a cota "390" foi lida como a base da lente de baixo. Medida por pixel
  na figura a 400 dpi, ela vai do chão ao **centro da lente amarela**. Centros
  reais: 530 / 390 / 250 mm, não 725 / 585 / 445.
- *Solução:* com 530 mm, a vermelha fica a 19,0° na largada e cabe até pitch
  ≈ 17°. Os 15° do desenho funcionam. `docs/fov-analise.md`.

**2. Quatro números do simulador do organizador divergem do regulamento.**
- Placa Ø300 (é Ø150), 3 classes (são 6), linha central amarela (é branca),
  ilha Ø0,57 (as cotas dão Ø1,4).
- *Solução:* o mapa é gerado pelo regulamento. ADR 001.

**3. Errei a largura das vias norte-sul na primeira versão do mapa.**
- *Causa:* usei 80 cm para todas as vias internas. Relendo a Figura 6, as vias
  ao lado do canteiro *vertical* têm cota "100 | 100"; só as do canteiro
  *horizontal* têm "80".
- *Solução:* parâmetros separados no gerador e teste que trava os dois valores.

**4. Primeira estimativa do FOV usava conta pinhole numa lente fisheye.**
- *Causa:* `f = (w/2)/tan(HFOV/2)` é pinhole. Para fisheye equidistante,
  `r = f·θ`.
- *Solução:* com 120° na diagonal, f ≈ 979 px → 96° H × 72° V, que bate com
  os ~92° × 75° estimados no CLAUDE.md.

**5. Render abaixo da meta: 395 passos/s em 320×240.**
- *Causa medida (perfil):* chão 0,46 ms; **sprites 1,5 ms (70 %)** — projeção
  por placa e lista refeita a cada quadro.
- *Solução:* sprites montados uma vez e projetados numa chamada vetorizada;
  linhas acima do horizonte fora do remap. 395 → **1.092 passos/s**. ADR 002.

**6. Seta de curva da placa desenhada para o lado errado.**
- *Causa:* sinal invertido no deslocamento do arco.
- *Solução:* reescrita; conferida visualmente contra a Figura 8.

### Decisões

- [ADR 001 — mapa pelo regulamento](adr/001-mapa-pelo-regulamento.md)
- [ADR 002 — câmera fisheye em NumPy e chão por grade](adr/002-camera-fisheye-e-chao-por-grade.md)

### Pendências abertas hoje

- **IMU:** o REV03 lista **BNO085**; o CLAUDE.md fala em **MPU6050**. Qual está
  no carro?
- Intrínsecos são estimados — calibrar com tabuleiro (F1).
- Perguntas à organização em `docs/regulamento.md`.

---

## 02/11/2026 (tarde) — Vista-Sim S1.1: cidade inteira, placas, iluminação e rotação de roda

Retorno do Guga sobre o primeiro vídeo: (1) o simulador precisa entender
**iluminação** — mesmo branca, ela muda com a oclusão do ambiente; (2) o carro
**não respeitou as placas**; (3) não havia **faixa de pedestre**; (4) **não
entrou na cidade**; (5) o carro tem que ser simulado **todo em diferença de
rotação**.

### Feito

- **Rotação de roda** (`car.py`): `env.step(rpm_esq, rpm_dir)`; PID por roda a
  100 Hz, zona morta, encoder de 440 ticks, raios ±0,5 % e patinação sorteados
  por episódio. `(v, ω)` virou só atalho. ADR 003.
- **Malha viária** (`roads.py`): 12 cruzamentos, 20 ruas de mão dupla, faixas,
  conversões e a pose de **toda** placa pela regra 3.5.2.
- **Regras de placa** (`rules.py`): obrigação, proibição, "sem placa = em
  frente" (política nossa). O oráculo decide com elas; o Vista vai importá-las.
- **Cenários** (`scenario.py`): rota pela cidade, placas que levam à rota
  (conferidas), distratoras, zebras, maquetes, público, lâmpadas. ADR 004.
- **Iluminação** (`lighting.py`): sombras de prédios, pessoas, placas e
  semáforo; sombra do próprio carro; rótulo de fração visível da placa.
- **Fiscal da rota** (`env.py` + `route.py`): `desrespeitou_placa`,
  `placa_respeitada`, `saiu_da_pista`, `rota_completa`…
- Oráculo novo: segue a rota decidindo pela placa e comanda rpm por roda.
- Demo com câmera + mapa visto de cima; 73 testes.

### Números

| Medida | Valor |
|---|---|
| Rotas sorteadas (200 sementes) | 25–66 m, 6–15 cruzamentos, **99 % entram no centro** |
| Oráculo nas 200 rotas | **200/200 completas**, 0 placa desrespeitada, 0 saída |
| `e_y` do oráculo (semente 139) | RMS 0,41 cm, máx. 2,78 cm |
| Rota da demo (semente 139) | 37 m, 11 cruzamentos (6 no centro), 10 placas, 93 s |
| Render só chão 320×240 | 837 passos/s |
| Render cidade inteira com luz 320×240 | 241 passos/s |
| `env.step` completo 410×308 | **204 passos/s = 6,8× tempo real** |
| Carro com roda 0,5 % maior, rpm iguais | desvia 29 cm em 5 m |

### Problemas, causas e soluções

**1. Com a cidade inteira, o simulador caiu para 77 passos/s.**
- *Causa medida (perfil):* faces de prédio projetadas uma a uma (2,5 ms),
  "está no carpete?" por polígono (1,4 ms), sombra do carro num grid 4×4 m com
  3 desfoques (1,5 ms), luz amostrada por quadro.
- *Solução:* faces numa projeção só; máscara do carpete de 1 cm; grid da sombra
  2×2 m com 1 desfoque e recalculado a cada 4 cm / 2°; luz do ambiente
  **assada na textura** uma vez por cenário; cores dos objetos pré-calculadas.
  77 → 204 passos/s no `env.step`. Ainda abaixo dos 1.000 do S1 com a cidade
  cheia — para gerar dados, 1 processo por núcleo.

**2. Placas caindo no asfalto nas vias em volta da ilha.**
- *Causa:* a via tem uma dobra; eu recuava 400 mm em linha reta usando o rumo
  da dobra. Depois, ao lado da ilha redonda, a borda reta da via deixa uma
  fresta de asfalto.
- *Solução:* 400 mm medidos ao longo da rua; e a placa é empurrada para fora
  até sair do asfalto ("tangenciando a calçada"). Teste em toda aproximação.

**3. "Gancho" na conversão para a via ao lado da ilha.**
- *Causa:* a faixa de saída começava em cima da dobra (rua de 1,0 m estreitando
  para 0,8 m); a Bézier herdava o rumo torto e passava do canto.
- *Solução:* dobra a 45° dentro do cruzamento, rumo da rua numa base de 40 cm,
  Bézier com controles proporcionais às pernas.

**4. O carro raspava a ponta do canteiro ou a borda nas conversões, mesmo
seguindo a trilha com 1,4 cm de erro.**
- *Causa:* o eixo é traseiro; com o eixo na trilha, a frente (34 cm adiante)
  balança para fora.
- *Tentativa que piorou:* terminar a curva 30 cm antes — fechou a curva e
  34 de 40 rotas saíram pela borda. Revertido.
- *Solução:* varri o contorno do carro pelas 96 conversões com o ponto de
  controle em várias posições: eixo 4 falhas, câmera 12, **centro do carro (14
  cm) 0**. O oráculo controla o centro (linearização de ponto fora do eixo) e
  a verdade de campo é medida nele. ADR 003.

**5. Mirar num alvo à frente cortava a curva em 7 cm.**
- *Causa:* perseguir um ponto L adiante numa curva de raio R corta ~L²/2R.
- *Solução:* velocidade tangente à trilha (adiantada do atraso de 0,1 s) +
  correção proporcional do erro lateral.

**6. Bug: 40 de 40 rotas falhando depois da troca de controle.**
- *Causa:* o laço que mede a curva à frente reaproveitava a variável `d` e
  sobrescrevia a distância do centro do carro (0,14 → 0,6 m).
- *Solução:* variável renomeada. 40/40 → depois 200/200.

**7. Rotas de 148–164 m com 40 cruzamentos estourando o tempo.**
- *Causa:* a exigência de "mesma aproximação, mesma manobra" fazia a volta à
  largada dar várias voltas.
- *Solução:* rota inteira limitada a 75 m (o regulamento não dá o tamanho).

**8. Odometria errando 18 m numa rota de 40 m.**
- *Causa:* patinação sorteada diferente em cada roda vira erro de rumo
  constante.
- *Solução:* patinação igual nas duas (é o mesmo piso); a assimetria vem da
  tolerância de raio. Ainda erra ~1–2,5 m no fim da rota — esperado sem IMU e
  sem calibrar o raio; o Vista não deve depender de odometria para posição.

### Decisões

- [ADR 003 — carro em rotação de roda](adr/003-carro-em-rotacao-de-roda.md)
- [ADR 004 — cenários, placas e iluminação](adr/004-cenarios-placas-e-iluminacao.md)

### Pendências abertas hoje

- O firmware do ESP32 vai receber **rpm por roda** ou `(v, ω)`?
- **IMU:** BNO085 (REV03) ou MPU6050 (CLAUDE.md)? Continua aberta.
- Zona morta, constante de tempo e ganhos do PID são estimados — medir no carro.
- Perguntas 8–12 em `docs/regulamento.md` (faixa de pedestre, cruzamento sem
  placa, proibição ambígua, fim da prova, maquetes).

---

## 02/11/2026 (noite) — Duas pendências fechadas

### Decidido pela equipe

- **ESP32 recebe rpm de cada roda.** É exatamente o contrato do `env.step`
  (ADR 003); nada muda no simulador.
- **IMU é MPU6050**, não o BNO085 do REV03. `config/car.yaml` atualizado com os
  números do datasheet.

### O que a troca muda

- **Sem magnetômetro e sem fusão pronta:** o rumo vem da integração do
  giroscópio z, que deriva com o viés. O BNO085 entregaria um quatérnio já
  fundido.
- **Viés de fábrica de até ±20 °/s** (tolerância do zero no datasheet). Sem
  calibrar, isso dá até 20° de erro por segundo. A largada ajuda: o carro
  fica parado no vermelho por alguns segundos — é a janela para medir o viés
  (média do giroscópio parado) antes de sair.
- Fundo de escala de ±250 °/s basta: o limite de giro do Vista é 3 rad/s ≈
  172 °/s.

### Pendências

- Posição da MPU6050 no carro: o `x = 0,217` é o lugar do BNO085 no REV03 —
  conferir.
- Viés residual depois de calibrar (`gyro_bias_residual_dps`) é estimativa —
  medir com o carro parado 60 s.

---

## 02/11/2026 (noite) — MPU6050 no simulador

Aprovado pela equipe: simular a IMU já, antes da S4.

### Feito

- `vista_sim/imu.py`: sensor MPU6050 (viés de fábrica ±20 °/s, escala ±3 %,
  ruído do datasheet, viés que anda, ADC de 16 bits, saturação ±250 °/s,
  acelerômetro com braço de alavanca) e a odometria que vai para o carro
  (deslocamento pelos encoders, rumo pelo giroscópio, viés calibrado parado).
- Telemetria com leitura bruta, viés estimado e se já calibrou; verdade com o
  viés real. Demo mostra o giroscópio e as duas odometrias. ADR 005.
- 79 testes.

### Números (40 rotas sorteadas)

| | só encoders | encoders + giro | + escala calibrada |
|---|---|---|---|
| erro de posição no fim (mediana / pior) | 1,84 / 9,31 m | 0,51 / 1,35 m | 0,19 / 0,77 m |
| maior erro de rumo (mediana / pior) | 24,5° / 108,7° | 5,9° / 39,3° | 2,2° / 9,3° |

Viés estimado no vermelho com erro < 0,02 °/s (rota da demo: −17,03 contra
−17,04 °/s reais). Custo da IMU: 0,09 ms por quadro.

### Problemas, causas e soluções

**1. O rumo girava ~6° com o carro parado na largada.**
- *Causa:* a odometria só considerava o carro "parado" depois de 0,3 s sem
  ticks; nesses 0,3 s iniciais integrava o viés ainda cru (até 20 °/s).
- *Solução:* na largada o carro está parado por definição — a odometria já
  começa no estado parado. Teste trava o rumo constante no vermelho.

### Pendências

- Calibrar o **fator de escala** no carro: girar no lugar N voltas e comparar
  com 360°·N. É o maior erro que sobra.
- Medir o viés que anda durante 2 min parado e a posição real da MPU6050.

---

## 02/11/2026 (noite) — Pipeline de treino para o Colab

A equipe aprovou treinar no Colab (100 unidades de computação): redes
pré-treinadas ajustadas com dados do simulador. ADR 006.

### Feito

- `tools/gerar_dataset.py`: o oráculo dirige rotas sorteadas, em paralelo, e o
  simulador grava imagens + rótulos (faixa, cruzamento, manobra, placas,
  semáforo). Câmera com montagem sorteada; ruído de recuperação; vistas
  deslocadas; semáforo desenhado em estado sorteado.
- `treino/`: rede de faixa (ResNet18 + manobra), dataset do detector (YOLO),
  avaliação com a rede dirigindo no simulador.
- `vista_sim/controle.py`: `PilotoFaixa`, a lei do oráculo escrita no
  referencial do carro, alimentada pela percepção — vai para o carro.
- `env.step(..., render=False)`, `env.rotulos_em(pose)`, `kappa_ahead` na verdade.
- `notebooks/treino_vista.ipynb`: do clone ao ONNX no Drive.

### Números

| Medida | Valor |
|---|---|
| `PilotoFaixa` com a verdade (40 rotas) | 40/40 completas, `e_y` máx. 3,0 cm |
| Oráculo com ruído de recuperação (40 rotas) | 40/40 completas, `e_y` p95 3,8 cm |
| Geração (8 episódios, 4 núcleos) | 11.389 imagens em 67 s, ~24 KB cada |
| Cobertura de `e_y` (p5–p95) | real −2,0…+2,5 cm; deslocadas −8,9…+14,8 cm |

### Problemas, causas e soluções

**1. Metade dos episódios com ruído saía da pista.**
- *Causa 1:* o alvo lateral do ruído pulava (ex.: +1,6 → −4,2 cm); o controle
  do centro do carro responde a um pulo com uma guinada de ~20° e a quina da
  frente tocava a borda.
- *Causa 2:* ±6 cm para a direita não cabe na via de 80 cm (o centro da faixa
  está a 20 cm do meio-fio; o carro tem 12 de meia largura).
- *Solução:* desvio desliza a 5 cm/s (inclusive quando some perto das
  conversões); até 3,5 cm para o meio-fio e 6 cm para a contramão. 23/40 →
  29/40 → 40/40.

**2. O `e_y` dos dados ficava em ±2,5 cm — a rede nunca veria o carro torto.**
- *Solução:* vistas deslocadas renderizadas com rótulos exatos (técnica do
  PilotNet, sem aproximação de imagem).

**3. 40 % das imagens descartadas no dataset do detector.**
- *Causa:* placas minúsculas e distantes contavam como "ambíguas".
- *Solução:* só é ambígua placa grande o bastante e meio escondida; minúscula é
  ignorada.

**4. Semáforo vermelho/amarelo raro nos dados.**
- *Causa:* só existe na espera da largada (poucos quadros, um ângulo só).
- *Solução:* depois da largada o DESENHO do semáforo é sorteado; o carro
  obedece ao real.

**5. Risco no Colab: memória compartilhada dos workers.**
- *Causa:* lotes float32 de 236 MB × 12 workers.
- *Solução:* workers mandam uint8; aumento de dados em lote na GPU.

### Pendências

- Rodar o notebook no Colab e trazer `avaliacao_simulador.json`.
- Detector no laço (manobra pela placa detectada + `rules.decide`).
- Imagens reais da pista caseira para ajuste fino quando o carro existir.

---

## 02/11/2026 (noite) — Primeiro treino da rede de faixa no Colab

Rodado pela equipe com o notebook (`EPOCAS_FAIXA = 20`). Valores **lidos do
gráfico** do notebook; os exatos estão em `faixa_historico.json` no Drive.

| Validação (episódios nunca vistos) | época 0 | época 20 |
|---|---|---|
| perda (treino / validação) | 0,31 / 0,16 | ~0,015 / ~0,049 |
| erro de `e_y` na faixa | 1,1 cm | ~0,3 cm |
| erro de `e_y` dentro do cruzamento | 1,95 cm | ~1,0 cm |
| erro de `e_ψ` na faixa | 1,2° | ~0,65° |
| erro da distância ao cruzamento | 11,6 cm | ~3,6 cm |

Leitura:

- A validação estabiliza por volta das épocas 12–15 e **não sobe** — sem
  sobreajuste destrutivo, mas sem ganho com mais épocas.
- Perda de treino 3× menor que a de validação: o próximo ganho vem de **mais
  episódios** (variedade), não de mais épocas.
- No cruzamento o erro é ~3× o da faixa: sem linha pintada, a rede usa a
  manobra e a geometria do cruzamento. É onde o giroscópio deve ajudar.
- São números simulador → simulador. O que decide é a rede dirigindo
  (`avaliacao_simulador.json`) e, depois, imagens reais.

### Pendências

- Resultado do YOLO (mAP50 por classe) e da rede dirigindo no simulador.

---

## 02/11/2026 (noite) — Detector e rede de faixa dirigindo: o primeiro resultado de verdade

Rodado pela equipe no Colab (A100), 150 episódios.

### Detector (YOLO11n, validação em episódios nunca vistos)

| | P | R | mAP50 | mAP50-95 |
|---|---|---|---|---|
| todas | 0,916 | 0,868 | **0,948** | **0,827** |
| semáforo vermelho / amarelo / verde | 0,98–0,99 | 0,99 | 0,994–0,995 | 0,92–0,93 |
| `seguir_em_frente` | 0,924 | 0,746 | 0,895 | 0,768 |
| `proibido_virar_direita` | 0,942 | 0,716 | 0,913 | 0,736 |

Semáforo resolvido no simulador. As placas que mais escapam (recall ~0,72–0,75)
são, provavelmente, as pequenas e distantes; precisão alta = quase não inventa
placa. Inferência: 1,0 ms por imagem na A100.

### Rede de faixa dirigindo (10 sementes nunca vistas; manobra e semáforo da verdade)

| | rotas completas | `e_y` RMS mediano |
|---|---|---|
| teto (mesmo piloto com a verdade) | **10/10** | 0,28 cm |
| rede de faixa | **4/10** | 1,75 cm |

Falhas: 5 × `desrespeitou_placa` (o carro saiu > 30 cm da trilha **dentro do
cruzamento** e entrou na rua errada — a manobra vinha da verdade, então não é
leitura de placa) e 1 × `saiu_da_pista`.

### Diagnóstico

Erro de 0,3 cm por imagem e 4/10 rotas é o sintoma clássico de **mudança de
distribuição**: os dados vêm do oráculo dirigindo bem; quando a rede dirige,
erra um pouco, chega a situações que nunca viu (sobretudo dentro do
cruzamento, sem linha pintada) e erra mais.

Experimento: o piloto com a verdade + ruído gaussiano no `e_y`/`e_ψ`
(20 sementes):

| ruído `e_y` (faixa / cruzamento) | sem filtro | com filtro complementar (odometria + giro) |
|---|---|---|
| 0,4 / 1,3 cm (≈ erro da rede na validação) | 17/20 | 16/20 |
| 0,8 / 2,5 cm | 8/20 | **14/20** |

- Ruído branco do tamanho do erro de validação ainda dá 17/20 — a rede (4/10)
  erra **mais** dirigindo do que na validação: confirma a mudança de
  distribuição.
- O controle do centro do carro amplifica ruído (1 cm de `e_y` ≈ 0,3 rad/s de
  giro); um filtro com o movimento do próprio carro segura isso quando o erro
  cresce.

### Próximo passo proposto (aguardando ok da equipe)

1. **DAgger**: a rede dirige no simulador, o oráculo assume quando ela se
   afasta, as imagens dessas situações entram no treino com o rótulo exato;
   2–3 rodadas.
2. **Filtro** de `e_y`/`e_ψ` com odometria + giroscópio entre a rede e o
   controle.

---

## 02/11/2026 (noite) — DAgger e filtro implementados; teste local do DAgger não melhorou

### Feito

- `FiltroFaixa` (encoders + giroscópio prevêem, a rede corrige), `PilotoRede`,
  DAgger no gerador (`--piloto-onnx`), ajuste fino a partir de checkpoint,
  validação dividida por bloco de sementes, seção de DAgger no notebook e
  opção de continuar de um modelo salvo. ADR 007.

### Teste local de uma rodada de DAgger (modelo FRACO: do zero, 96×128, CPU)

| | rotas completas (8) | progresso médio |
|---|---|---|
| antes, sem filtro | 0/8 | 11 % |
| antes, com filtro | 0/8 | 15 % |
| depois de 1 rodada (6.120 imagens, 75 % com a rede dirigindo) | 0/8 | 11 % |

**Não melhorou.** Olhando quadro a quadro a falha (semente 100000, conversão à
esquerda em T_ne), dentro do cruzamento a rede diz κ ≈ 0–0,2 (real 1,3),
`e_ψ` ≈ 0° (real −12° → −41°), "dentro?" 3 % (real: sim) e distância ~1–1,7 m
(real: já passou). Ela acha que ainda está numa reta antes do cruzamento: não
percebe que entrou nem que devia virar. O carro segue reto e cai na rua errada.

Dentro do cruzamento não há linha pintada: a trilha certa é uma curva
**invisível** que a rede teria de imaginar a partir da manobra. O modelo forte
do Colab erra muito menos, mas as 5 falhas dele também foram ali.

### Próximo passo proposto (aguardando ok)

**Atravessar o cruzamento por odometria + giroscópio**, usando o mapa:

1. o Vista sabe em que rua está seguindo o **grafo do mapa** com as próprias
   decisões (larga numa rua conhecida; cada manobra leva a uma rua conhecida) —
   isso também dá as saídas do cruzamento, que `rules.decide` precisa para as
   placas de proibição;
2. na entrada do cruzamento (rede: distância → 0), a trajetória da conversão
   vem do mapa, posicionada pelo `e_y`/`e_ψ` da rede na faixa (onde ela erra
   0,3 cm), e é seguida por encoders + giroscópio (~1,5 m: erro de ~1 cm);
3. na saída, a rede volta a mandar.

---

## 07/11/2026 — Atualização geral da documentação

Pedido da equipe: deixar o repositório inteiro em dia com o que foi feito.

### Feito

- **`docs/VISAO-GERAL.md`** (novo): o documento para quem chega agora.
  - objetivo e estratégia;
  - o carro;
  - a arquitetura do Vista, com o que vai para o carro e o que é só do simulador;
  - o simulador e o treino;
  - todos os resultados até aqui;
  - o problema do cruzamento e a proposta;
  - plano semana a semana até 04/12;
  - decisões e perguntas em aberto.

  O antigo `VISAO-GERAL.md` foi apagado em 02/11 junto com o código do carro
  errado; este o substitui.
- **README**: tabela de estado atualizada (detector, rede de faixa 4/10,
  filtro, DAgger, proposta do cruzamento), seção de treino com DAgger e
  `CONTINUAR_DE`, mapa do repositório, links para a visão geral e para o
  resumo de 02/11.
- **Resumo de 02/11**: pendências do Colab marcadas como feitas, com os
  resultados do detector, da rede dirigindo, do filtro e do DAgger.
- `.gitignore`: removida a referência a `data/README.md` (apagado em 02/11).

### Estado

- 89 testes.
- Todo o trabalho continua no branch `claude/autonomous-car-repo-data-lzwn7t`;
  o `main` só tem o commit inicial.
- Em aberto: a proposta de atravessar o cruzamento por odometria + localização
  no mapa (aguardando ok).

---

## 09/11/2026 — Primeira medida real na Jetson: YOLO11n a 10–25 ms

Informado pela equipe: num projeto anterior (detecção em soja), o **mesmo
modelo YOLO11n** rodou na Jetson em **10–25 ms** por imagem. Condições ainda
não registradas: tamanho de imagem (provavelmente 640, o padrão), formato
(TensorRT/ONNX/PyTorch), se inclui pré-processamento e NMS, e qual Jetson.

### Modelos do Vista (contados)

| | Rede de faixa | Detector |
|---|---|---|
| Modelo | ResNet18 + cabeça (manobra) | YOLO11n |
| Parâmetros | 11.309.893 (cabeça: 133.381) | 2.598.995 |
| Custo por imagem | 5,6 GFLOPs (240×320) | 2,7 GFLOPs (416×416); 6,4 a 640×640 |
| ONNX FP32 | ~45 MB | 10,1 MB |

### Orçamento de tempo

A câmera a 30 fps dá 33 ms por quadro. Se o detector custar 10–25 ms e a rede
de faixa algo parecido (estimativa pelo custo; não medido), as duas a cada
quadro podem passar de 33 ms. Proposta:

- **Rede de faixa a cada quadro** (30 Hz).
- **Detector a cada 2–3 quadros** (10–15 Hz). A 0,5 m/s são 3–5 cm entre
  detecções; a placa aparece a ~1,5 m do cruzamento.

Medir as duas em TensorRT FP16 na Orin Nano do carro, com pré-processamento,
está no plano de 14–20/11.

### 09/11 — condições da medida (informadas pela equipe)

- Imagem **640×640**; precisão **FP16 ou FP32** (não lembrado); tempo do
  **pipeline inteiro** (pré-processamento + inferência + caixas/classes);
  **mesma Orin Nano** do carro.

Leitura: é o pior caso para o Vista. Com 416×416 o detector deve ficar abaixo
de 10–25 ms (não 2,4× menos: parte do tempo é fixa). A rede de faixa tem custo
parecido com o YOLO a 640 e não tem pós-processamento de caixas → deve ficar
≤ 10–25 ms. Caso típico: ~20 ms as duas juntas (cabe em 33 ms); pior caso:
~45 ms (não cabe) → detector a cada 2–3 quadros como padrão seguro. A
variação (10 a 25 ms) pesa no controle; anotar o modo de energia (`nvpmodel`)
na próxima medida — sem alterá-lo sem a equipe.

---

## 09/11/2026 (noite) — Vista completo: cruzamento pelo mapa, detector no laço

### Decisões da equipe (perguntas e respostas)

| Tema | Resposta |
|---|---|
| Cruzamento | **odometria + mapa** (proposta aceita) |
| Placas | **Vista completo**: detector + regras no laço, só sensores do carro |
| Velocidade | completar a **0,5 m/s** e depois subir; dados em várias velocidades |
| Sem placa | **em frente** — política confirmada |
| Carro | ainda em montagem |
| Organização | as perguntas **não** serão enviadas |
| Repositório | fica público |
| `main` | abrir PR do branch |

Registrado no ADR 008 e em `docs/regulamento.md`.

### O que foi feito — pacote `vista/`

- `mapa.py` — `LocalizadorGrafo`: rua atual pelo grafo (o Vista sabe a rua
  seguinte porque ele mesmo decide a conversão); pose prevista pela
  odometria e amarrada à faixa do mapa pela rede.
- `cruzamento.py` — `ManobraOdometria`: `fim da faixa → net.movement → começo
  da faixa de saída`, seguido com a lei do oráculo e a pose da odometria.
- `placas.py` — detector ONNX (letterbox e NMS próprios), detector perfeito
  para testes, e o `LeitorPlacas`.
- `piloto.py` — `VistaPiloto`: aguardando verde → faixa ↔ cruzamento.
- `treino/avaliacao.py::rodar_vista` (+ `tools/avaliar_vista.py`): avalia em
  paralelo, com cada parte trocável pela verdade (`FaixaVerdade`,
  `FaixaRuidosa`, `DetectorVerdade`).

### Problemas e soluções, na ordem

1. **Script de avaliação travado** (>10 min para 4 rotas): `spawn` sem
   `if __name__ == "__main__"` no script de teste → cada processo filho
   reabria o pool. Só no script (o notebook não sofre disso).
2. **Rumo 2° torto na entrada do cruzamento**, mesmo com a verdade: o
   filtro prevê o rumo com `κ` da rede, que é a média dos **próximos 40 cm**
   — chegando numa conversão ele já "vê" a curva e a previsão gira antes da
   hora. Com ganho 0,12 o erro estacionário é ~2°. Solução: a previsão usa a
   curvatura **do mapa onde o carro está** (`kappa_atual`). O v1 tinha o
   mesmo viés.
3. **Placa errada (distratora) atribuída à minha aproximação**: a janela em
   volta do pixel previsto pegava a placa de outro cruzamento na mesma
   direção, bem menor. Solução: janela vertical menor (a altura da placa é
   conhecida), escolher a detecção **mais perto** do previsto e exigir
   tamanho entre 0,6× e 1,7× do previsto.
4. **Placa da rua curta nunca vista**: a placa do cruzamento seguinte aparece
   **durante a curva** anterior, quando o leitor ainda olhava a aproximação
   velha. E, pior, perto ela sai pelo topo da imagem: com a câmera a 15,6 cm
   e 15° para baixo, o centro da placa (47,5 cm) só cabe a mais de ~0,8 m.
   Nas ruas de ~1 m do centro, ela quase nunca aparece na própria aproximação.
   Solução: o leitor atribui **cada** placa detectada à aproximação do mapa
   que melhor a explica (todas, sempre), e os votos se acumulam desde longe.
   Detector roda também dentro do cruzamento.
   Medido sem render (24 rotas, 233 placas de rota, sem oclusão):

   | Pitch | legível em algum momento | na própria aproximação | nunca |
   |---|---|---|---|
   | 15° | 98,7 % | 66,5 % | 1,3 % |
   | 10° | 98,7 % | 73,8 % | 0,9 % |
   | 7° | 99,1 % | 83,7 % | 0,9 % |
   | 5° | 99,1 % | 85,0 % | 0,9 % |
   | 0° | 99,1 % | 85,4 % | 0,0 % |

   A análise de FOV de 02/11 dizia que o semáforo não restringe o pitch; as
   placas restringem. Recomendação para a equipe: avaliar 5–7°.
5. **Proibição ambígua virava "em frente"**: com "proibido seguir em frente"
   lido num cruzamento de 4 vias, `decide` dá erro (duas saídas), e o código
   caía para "sem placa = em frente" — justamente a proibida. Corrigido: se
   houve placa lida, vai para a reserva provisória (frente → direita →
   esquerda, a que ela não proíbe), registrada como `alternativa`.
6. **Carro saiu da pista logo depois de um cruzamento (semente 100000)**:
   na saída da curva a pose estava 5 cm e 2,6° errada — é a escala do
   giroscópio (±3 % na MPU6050; 3 % de 90° = 2,7°). A rede, de volta,
   mandou corrigir os 5 cm de uma vez: `k·e_y` = 0,23 m/s → 1,6 rad/s de
   giro, a frente varreu para fora. Duas correções:
   - **escala do giroscópio calibrada a cada curva**: giro contado ×
     giro da pose amarrada à faixa antes e depois; mínimos quadrados com
     prior fraco. Na 100000: 0,977 na primeira curva;
   - **ângulo de aproximação limitado**: `k·e_y` ≤ 0,35·v (~20°), no
     PilotoFaixa e na curva do mapa.

### Resultados (simulador, 0,5 m/s)

| Configuração | Rotas | Decisões certas |
|---|---|---|
| Teto do Vista (rede e detector perfeitos), 12 rotas | **11/12** | 118/119 |
| Antes das correções 2–6 (8 rotas) | 3/8 → 6/8 | |
| Rede local de teste + detector perfeito (8 rotas) | 0/8 | — a rede erra 6–9 cm na própria faixa; não mede nada |

A única falha do teto: semente 100007, placa da rua curta vista só ~0,3 s,
cortada no topo da imagem → sem placa → em frente → errado.
7. **Teste do DAgger quebrou** (só "rede", o oráculo nunca assumia): (a) o
   gerador sorteia o *desenho* do semáforo depois do verde, para o detector
   ver os 3 estados — e o Vista, que agora larga pelo semáforo que vê, nunca
   via dois verdes seguidos. No DAgger o sorteio fica desligado. (b) Com uma
   rede enviesada de propósito, o carro saía do tapete antes de chegar a
   6 cm: à direita sobram 8 cm até o meio-fio. O oráculo agora assume a
   3,5 cm desse lado (como o limite do ruído dele). (c) O limite de
   aproximação tinha piso de 0,1 m/s — parado, o carro girava no lugar
   para corrigir; agora é proporcional a v.
8. **Oráculo a 0,8 m/s: 0/40** (sai da pista oscilando, período ~0,5 s).
   Com 0,1 s de atraso e o ponto controlado a 14 cm do eixo, k = 4 é demais
   nessa velocidade. Testado: k = 3 → 20/20, 2,5 → 20/20, 2,0 → 20/20 (erro
   maior). Regra: **k·v ≤ 2,4** — nada muda até 0,6 m/s; a 0,8 dá k = 3.
   Oráculo: 40/40 a 0,5 (números idênticos aos de antes) e **40/40 a 0,8**.
   Vale para o PilotoFaixa e a curva do mapa também.

### Resultados finais do dia

| Configuração | Rotas | Decisões certas |
|---|---|---|
| Teto do Vista, 12 rotas, 0,5 m/s | **11/12** | 118/119 |
| Verdade + ruído da rede (0,8/2,5 cm, e_ψ 0,7/1,5°, κ 0,15, dist 3,6 cm), 20 rotas | **18/20** | 197/199 |
| *v1 com o mesmo ruído + filtro, manobra da verdade (02/11)* | *14/20* | — |
| Teto do Vista a 0,8 m/s, 8 rotas | **7/8** | 82/83 |
| PilotoFaixa com a verdade (v1), 40 rotas — regressão | 40/40 | — |

Todas as falhas: placa de rua curta nunca lida (100007 e 100013). Nenhuma de
direção. 99 testes.

Notebook novo: `notebooks/treino_vista_v2.ipynb` — parte dos modelos de
02/11, mede teto / rede / rede + YOLO / v1 nas mesmas rotas, ajusta a rede
com dados em 0,35–0,80 m/s, DAgger com o Vista dirigindo, varre a velocidade
(0,5 → 0,8) e tem a opção `PITCH_GRAUS` para o experimento da câmera.

