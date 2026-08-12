# Hardware — montagem, ligações e escolhas

## Lista de materiais (BOM)

| # | Item | Especificação | Obs. |
|---|---|---|---|
| 1 | Jetson Orin Nano | Devkit, 8 GB | Cérebro. Consome 7–15 W |
| 2 | Câmera CSI | **IMX219 8 MP 3280×2464, lente 120°, foco ajustável** | ✅ definida — ver "a câmera do projeto" |
| 3 | ESP32 | DevKitC / WROOM-32 | Controle em tempo real |
| 4 | Chassi RC 1:10 | Direção Ackermann + ESC + motor | ≤500×250 mm |
| 5 | Servo de direção | Metálico, ≥6 kg·cm, 4.8–6 V | Plástico folga e a direção fica imprecisa |
| 6 | ESC | Bidirecional, BEC 6 V/3 A | Neutro em 1500 µs |
| 7 | Ultrassônicos | HC-SR04 ×3 | Ver "sensores de distância" |
| 8 | Bateria de tração | LiPo 2S 5000 mAh | Motor + servo |
| 9 | Bateria do Jetson | Power bank USB-C PD 5 V/5 A ou 3S + regulador | **Alimentação separada** |
| 10 | Conversor DC-DC | 5 V / 5 A | Se não usar power bank |
| 11 | Botão + LED RGB | Botão NA, LED WS2812 | Armar/desarmar e status sem SSH |
| 12 | Cabo USB | USB-A ↔ micro/USB-C, curto e blindado | Jetson ↔ ESP32 |
| 13 | Cartão/SSD | NVMe 256 GB+ | Dataset cresce rápido — ver cálculo abaixo |

### Alimentação: separar as baterias

Motor e ESC geram picos de corrente que derrubam a tensão. Se o Jetson estiver
no mesmo barramento, ele **reinicia no meio da volta** — falha clássica e
frustrante porque só aparece com o carro andando.

```
LiPo 2S ──┬── ESC ── motor
          └── BEC 6 V ── servo + ESP32(Vin)

Power bank USB-C PD ── Jetson Orin Nano
```

O **GND do ESP32 e o GND do Jetson ficam unidos pelo cabo USB de dados**, o que
já dá a referência comum necessária para a serial. Não ligue os positivos.

## Ligações do ESP32

Pinos definidos em `firmware/esp32/include/config.h`. Alterou aqui, altere lá.

| Função | GPIO | Observação |
|---|---|---|
| Servo de direção (PWM) | 18 | LEDC, 50 Hz |
| ESC / tração (PWM) | 19 | LEDC, 50 Hz |
| HC-SR04 #0 TRIG / ECHO | 25 / 26 | frontal esquerdo (−35°) |
| HC-SR04 #1 TRIG / ECHO | 32 / 33 | frontal central |
| HC-SR04 #2 TRIG / ECHO | 27 / 14 | frontal direito (+35°) |
| Leitura de bateria (ADC) | 34 | divisor 10 kΩ / 3.3 kΩ, **entrada só de ADC** |
| Rádio RC — canal direção | 4 | opcional, só coleta de dados |
| Rádio RC — canal aceleração | 5 | opcional, só coleta de dados |
| LED de status (WS2812) | 2 | |
| Botão de armar | 15 | com `INPUT_PULLUP`, botão para o GND |
| Serial com o Jetson | USB (UART0) | 115200 8N1 |

> ⚠️ **ECHO do HC-SR04 é 5 V e o ESP32 é 3.3 V.** Use divisor resistivo
> (1 kΩ + 2 kΩ) em cada pino ECHO ou o GPIO morre. TRIG pode ir direto (3.3 V
> é suficiente para disparar).

## Sensores de distância: por que ultrassônico e não capacitivo

Vocês cogitaram sensores capacitivos. Vale registrar a diferença antes de
comprar:

| | Capacitivo | Ultrassônico (HC-SR04) | ToF (VL53L0X/L1X) |
|---|---|---|---|
| Alcance útil | **1–30 mm** | 20 mm – 4 m | 30 mm – 2/4 m |
| Mede distância? | Não (liga/desliga) | Sim | Sim, preciso |
| Taxa | alta | ~20 Hz por sensor | 50 Hz |
| Custo | baixo | muito baixo | médio |

Sensor capacitivo é de **proximidade por contato quase físico** — detecta a mão
a poucos milímetros. A 1 m/s o carro percorre 30 mm em 30 ms; quando o
capacitivo disparasse, a batida já teria acontecido. **Não serve para desvio de
obstáculo em movimento.** Ultrassônico é a escolha certa para essa faixa de
preço, e o VL53L1X é o upgrade natural se quiserem precisão e taxa maiores.

Por isso o código trata "sensor de distância" de forma genérica
(`distance_sensors.type` em `config/vehicle.yaml`): trocar HC-SR04 por ToF
depois muda o firmware do sensor, não o protocolo nem o dataset.

**Cuidado com crosstalk:** três HC-SR04 disparados juntos escutam o eco um do
outro. O firmware dispara em **rodízio**, um por vez, ~20 ms cada → ciclo
completo a ~16 Hz. Não tente acelerar isso disparando em paralelo.

Uma limitação real a considerar: o carpete e as maquetes da minicidade são
absorventes e podem devolver eco fraco; superfícies em ângulo agudo refletem o
som para longe do sensor. Ultrassônico serve para **não bater**, não para
navegar. **A navegação é por câmera** — o regulamento inclusive sugere isso, já
que "não há garantia de funcionamento de LIDAR ou GPS".

## A câmera do projeto

**Módulo IMX219 8 MP (3280 × 2464), lente 120° com foco ajustável, para
Jetson Nano / Orin Nano (conector CSI-2, 15 pinos).**

É a câmera que temos, e a análise abaixo é feita em cima dela. A conclusão
curta, para quem não vai ler a seção inteira:

> ✅ Dá para fazer MASTER com esta câmera — **desde que se capture em
> 1640 × 1232, não em 1280 × 720.** Capturar em 720p aqui é perda dupla:
> menos pixels *e* menos campo de visão.

### Modos do sensor — a pegadinha do IMX219

O `nvarguscamerasrc` expõe cinco modos, e **eles não têm o mesmo campo de
visão**. Três deles são recorte do sensor, não redução:

| Modo | FPS | Campo de visão | Observação |
|---|---|---|---|
| 3280 × 2464 | 21 | **completo** | Máximo detalhe; 21 fps é pouco para condução |
| 3280 × 1848 | 28 | largura total, recorte vertical | 16:9 |
| 1920 × 1080 | 30 | **RECORTADO** | Perde parte dos 120° |
| **1640 × 1232** | **30** | **completo** (binning 2×2) | ✅ **é o nosso modo** |
| 1280 × 720 | 60 | **RECORTADO** | Perde bastante FOV |

`1640 × 1232` é exatamente metade de `3280 × 2464` — ou seja, binning 2×2 de
todo o array. Mantém os 120° inteiros, entrega 30 fps e ainda tem o bônus de
que o binning **junta 4 fotodiodos por pixel**, o que melhora bastante a
imagem sob a luz fraca de ginásio.

Verifique os modos da sua placa com:

```bash
gst-inspect-1.0 nvarguscamerasrc
# ou, mais direto, olhe o log ao abrir a câmera:
robocar camera -v
```

### Tamanho da placa em pixels

```
px = (150 mm / (2 · d · tan(HFOV/2))) · largura_em_px
```

Estes módulos costumam anunciar o FOV na **diagonal**. Com 120° diagonais num
sensor 4:3, o horizontal fica em ~108°. Como a especificação do vendedor nem
sempre deixa claro, a tabela traz os dois casos — o pior deles é o que importa
para planejar:

| Captura | HFOV | 1,0 m | 1,5 m | 2,0 m | 2,5 m | 3,0 m |
|---|---|---|---|---|---|---|
| **1640 × 1232** | 108° (120° diag.) | 89 px | 60 px | **45 px** | 36 px | 30 px |
| **1640 × 1232** | 120° (pior caso) | 71 px | 47 px | **36 px** | 28 px | 24 px |
| 1280 × 720 *(recortado)* | 120° | 55 px | 37 px | **28 px** | 22 px | 18 px |
| 3280 × 1848 | 120° | 142 px | 95 px | 71 px | 57 px | 47 px |

Um classificador de setas trabalha com folga a partir de **~32 px**.

Lendo a linha do pior caso (`1640 × 1232` com 120° horizontais): a placa passa
dos 32 px a partir de **~2,2 m** e chega a 71 px a 1 m. Como ela fica 400 mm
antes da esquina, detectar a 2 m deixa **~1,6 m de percurso** para confirmar a
leitura em vários quadros, desacelerar e entrar na curva — a 1,5 m/s, mais de
1 segundo, ou 20 a 30 quadros. É apertado, mas funciona.

Em `1280 × 720` a mesma placa só passa de 32 px a ~1,7 m, e o modo ainda é
recortado. É a diferença entre dar certo e não dar.

### Se o detector de placas sofrer

Duas saídas, em ordem de custo:

1. **Colete o dataset de placas em `3280 × 1848` @ 28 fps.** Esse modo mantém a
   largura total do sensor (portanto os 120° horizontais inteiros) e só corta
   na vertical — parte que já descartamos no recorte. Dobra a resolução da
   placa e o custo em disco não importa, porque o dataset de placas são ~20
   minutos de filmagem, não 8 horas. Basta ajustar `camera.capture` durante
   essa coleta específica.
2. **Rodar o detector em `3280 × 1848` também na prova**, aceitando 28 fps. A
   rede de direção continua barata (ela usa 200 × 66 de qualquer forma); quem
   paga a conta é só o detector, e ele não precisa rodar a cada quadro — 5 a
   10 Hz é suficiente para uma placa que fica visível por mais de um segundo.

> Quer eliminar a dúvida do HFOV em 5 minutos? Cole uma folha A4 na parede
> (297 mm de largura, na horizontal), posicione a câmera a exatamente 1,00 m e
> rode `robocar camera --snapshot fov.jpg`. Meça a largura da folha em pixels
> na imagem e calcule:
> `HFOV = 2 · atan( (largura_px_total / largura_folha_px) · 297 / 2000 )`.
> Anote o resultado em `config/camera.yaml`.

### O lado bom dos 120°

Grande angular não é só desvantagem — para **manutenção de faixa** ela ajuda:
em curva fechada, a borda interna da rua continua no quadro, enquanto uma
lente estreita a perde justo quando ela mais importa. É por isso que 120° é um
meio-termo razoável, muito melhor que os 160° dos kits genéricos (que dariam
8 px na placa a 2 m e inviabilizariam o MASTER).

### Distorção de barril — cuidado específico desta lente

A 120°, as bordas do quadro têm distorção de barril perceptível. E a placa vive
justamente na borda direita. Consequências práticas:

1. **A placa aparece deformada** (esticada/curvada) conforme se aproxima e
   migra para a borda. O classificador precisa ver isso no treino — o que
   acontece de graça, já que o dataset é coletado com esta mesma câmera. **Não
   treine com imagens de placas baixadas da internet**: elas não têm a
   distorção que a nossa câmera produz.
2. Para geometria (estimar distância da placa), vale calibrar e retificar:
   `robocar calib camera` grava os coeficientes em
   `config/calib/camera_intrinsics.yaml`.
3. Para a rede de direção, **não retifique**. Ela aprende a relação
   pixel → esterço com a distorção incluída, e retificar só adicionaria custo
   de CPU e uma fonte de erro.

### Foco ajustável — travar depois de ajustar

O foco desta lente é uma rosca que gira. Isso é útil (dá para focar na
distância que interessa) e perigoso: **um esbarrão desfoca a câmera e o
dataset inteiro daquela sessão vira lixo**, muitas vezes sem ninguém perceber
na hora.

Procedimento:

1. Foque com o carro apontado para a pista, mirando algo a **~2 m** — a
   distância onde as placas precisam ser lidas.
2. Use o medidor de nitidez embutido, que mostra um número ao vivo:
   ```bash
   robocar camera --focus
   ```
   Gire a rosca devagar até o número **parar de subir**, depois recue até o
   pico. Ele é a variância do laplaciano: quanto maior, mais nítido.
3. **Trave a rosca** com uma gota de esmalte de unha ou trava-rosca (Loctite
   243) na junção lente/corpo. Não use cola forte — pode ser preciso reajustar.
4. Marque a posição com um traço de caneta permanente atravessando lente e
   corpo: um traço desalinhado denuncia o desfoque de longe.
5. Confira o número de nitidez no início de **toda** sessão de coleta. Se caiu
   muito em relação ao valor anotado, pare e reajuste antes de gravar.

### Onde a placa aparece no quadro

Placa: borda inferior a 400 mm, diâmetro 150 mm → centro a **475 mm** do chão.
Câmera a **180 mm**, apontada para o horizonte. Com VFOV de ~92° (120° diag.):

| Distância | Ângulo acima do horizonte | Altura no quadro (0 = topo) |
|---|---|---|
| 0,5 m | 30,5° | 0,22 |
| 1,0 m | 16,4° | 0,36 |
| 2,0 m | 8,4° | 0,43 |
| 3,0 m | 5,6° | 0,45 |

Repare que a lente grande angular **comprime os ângulos**: a placa fica só um
pouco acima do centro, não lá no alto do quadro como aconteceria com uma lente
estreita. Por isso `roi_signs` em `config/camera.yaml` vai de 10% a 60% da
altura (e não de 5% a 60%, que era o valor calculado para uma lente de 62°).

Horizontalmente, o regulamento garante a placa **sempre à direita**, então o
recorte cobre de 35% a 100% da largura. Isso corta metade dos pixels e, mais
importante, elimina o falso positivo vindo do público e das maquetes do lado
esquerdo.

## Montagem da câmera — o detalhe que arruína datasets

A rede aprende a relação **pixel → ângulo de esterço**. Se a câmera mudar de
posição entre a coleta e a prova, tudo o que foi aprendido se desloca junto.

- Fixe em suporte **rígido** (impressão 3D ou alumínio), sem parafuso frouxo.
- Marque a posição com tinta/adesivo depois de calibrar.
- Anote altura e inclinação em `config/camera.yaml` e no `session.json`.
- **Se mexer na câmera, os dados anteriores perdem validade.** Trate como uma
  quebra de versão do dataset: registre em `notes` e considere recoletar.
- Amortecimento contra vibração ajuda, mas **rigidez importa mais**: uma
  câmera "flutuando" em espuma balança em curva e injeta ruído no rótulo.

## Dimensionamento de disco

Gravar em `1640 × 1232` custa mais que em 720p, e esse é o preço de conseguir
ler as placas. Frame JPEG q90 nessa resolução ≈ **280 kB**. A 20 Hz:

| Duração | Frames | Disco (1640×1232 q90) | Comparação: 1280×720 |
|---|---|---|---|
| 1 min | 1 200 | ~340 MB | ~180 MB |
| 10 min | 12 000 | ~3,4 GB | ~1,8 GB |
| 1 h | 72 000 | ~20 GB | ~10,8 GB |
| 8 h (dia de treino, 04/12) | 576 000 | **~160 GB** | ~86 GB |

Consequências práticas:

- **Planeje 200 GB livres** para 04/12, ou esvazie o disco durante o dia
  (backup no HD externo entre as sessões — que é o procedimento recomendado de
  qualquer forma).
- Na prática ninguém grava 8 h contínuas; ~3 h de gravação efetiva ≈ 60 GB.
- Se o espaço apertar, `capture.image.quality: 85` economiza ~25% com perda
  visual desprezível. **Não desça abaixo de 80** — o artefato de JPEG começa a
  comer justamente a borda da placa e o contorno da faixa branca.
- **NVMe, não cartão SD.** A 20 Hz nessa resolução são ~5,6 MB/s de escrita
  contínua; cartão SD não sustenta isso com o resto do sistema rodando e a fila
  do gravador começa a derrubar frames (o contador `descartados_fila` acusa).

`robocar doctor` mostra o espaço livre e estima quantos minutos de gravação
cabem. Rode antes de sair de casa.

## Checklist de bancada antes de subir na pista

1. `robocar link --check` → `$PONG` respondendo, latência < 5 ms.
2. **Carro no cavalete, rodas no ar.** `robocar calib steering` → centro,
   batente esquerdo e direito. Anote em `config/vehicle.yaml`.
3. Ainda no cavalete: teste do failsafe — desconecte o USB e confirme que o
   motor vai a neutro em < 300 ms. **Não pule este teste.**
4. `robocar camera --preview` → foco, exposição, sem tremor.
5. `robocar record --dry-run` → confirma taxa de 20 Hz sem frames perdidos.
