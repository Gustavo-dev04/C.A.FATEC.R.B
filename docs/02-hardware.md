# Hardware — montagem, ligações e escolhas

## Lista de materiais (BOM)

| # | Item | Especificação | Obs. |
|---|---|---|---|
| 1 | Jetson Orin Nano | Devkit, 8 GB | Cérebro. Consome 7–15 W |
| 2 | Câmera CSI | IMX219 (Raspi Cam v2) ou IMX477 | **Ver "escolha da lente" abaixo** |
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

## Escolha da lente da câmera — a conta que decide MASTER

Grande angular (160°) é o padrão em kits de carrinho autônomo e seria uma
escolha ruim aqui. A razão é a placa de trânsito.

Tamanho aparente de uma placa de **150 mm** em pixels:

```
px = (150 mm / (2 · d · tan(HFOV/2))) · largura_em_px
```

| Lente | Captura | 1,0 m | 2,0 m | 3,0 m | 4,0 m |
|---|---|---|---|---|---|
| IMX219 padrão, HFOV 62° | 1280×720 | **159 px** | **79 px** | 53 px | 40 px |
| IMX219 padrão, HFOV 62° | 640×360 | 80 px | 40 px | 26 px | 20 px |
| Grande angular, HFOV 160° | 1280×720 | 17 px | 8 px | 6 px | 4 px |

Um classificador de setas precisa de **~32 px** para funcionar com folga. Com
grande angular, a placa só chega a 32 px quando o carro está a ~0,5 m dela — e
ela está a 400 mm do cruzamento, ou seja, **tarde demais para frear e virar**.

**Recomendação:**
- Lente padrão (~62–78° HFOV), captura em **1280×720**.
- Direção usa `roi_driving` reduzido a 200×66 → barato.
- Placas usam `roi_signs` em resolução nativa → detalhe preservado.

Se a lente estreita apertar demais a visão da rua nas curvas fechadas, a saída
é **duas câmeras** (uma grande angular para direção, uma estreita para placas),
não abrir o ângulo da única câmera.

### Onde a placa aparece no quadro

Placa: borda inferior a 400 mm, diâmetro 150 mm → centro a **475 mm** do chão.
Câmera a **180 mm**. Ângulo vertical até o centro da placa:

| Distância | Ângulo acima do horizonte |
|---|---|
| 1,0 m | 16,4° |
| 2,0 m | 8,4° |
| 3,0 m | 5,6° |

Com VFOV de ~48° (IMX219), tudo isso cai confortavelmente **na metade superior
do quadro** — e, como o regulamento garante que a placa está **sempre à
direita**, o recorte `roi_signs` (topo 5%–60%, direita 35%–100%) contém a placa
em qualquer distância útil. Menos pixels para processar e muito menos falso
positivo vindo do público e das maquetes do lado esquerdo.

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

Frame JPEG 1280×720 q90 ≈ **150 kB**. A 20 Hz:

| Duração | Frames | Disco |
|---|---|---|
| 1 min | 1 200 | ~180 MB |
| 10 min | 12 000 | ~1,8 GB |
| 1 h | 72 000 | ~10,8 GB |
| 8 h (dia de treino, 04/12) | 576 000 | **~86 GB** |

Na prática não se grava 8 h contínuas, mas **planeje 100 GB livres** para o dia
de treino. NVMe, não cartão SD: o SD não sustenta 3 MB/s de escrita contínua
com o resto do sistema rodando e vai começar a derrubar frames. Verifique com
`robocar doctor` antes de sair de casa.

## Checklist de bancada antes de subir na pista

1. `robocar link --check` → `$PONG` respondendo, latência < 5 ms.
2. **Carro no cavalete, rodas no ar.** `robocar calib steering` → centro,
   batente esquerdo e direito. Anote em `config/vehicle.yaml`.
3. Ainda no cavalete: teste do failsafe — desconecte o USB e confirme que o
   motor vai a neutro em < 300 ms. **Não pule este teste.**
4. `robocar camera --preview` → foco, exposição, sem tremor.
5. `robocar record --dry-run` → confirma taxa de 20 Hz sem frames perdidos.
