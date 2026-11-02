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
