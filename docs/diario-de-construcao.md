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
