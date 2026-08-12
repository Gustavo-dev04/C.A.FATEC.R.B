# Coleta de dados — o procedimento

Esta é a atividade mais importante do projeto agora. Modelo de condução é
limitado pelo dado, não pela arquitetura: uma PilotNet com 6 horas de dado bom
supera qualquer rede grande com 40 minutos de dado ruim.

---

## Antes de qualquer sessão

```bash
robocar doctor          # dependências, disco, porta serial, câmera
robocar link --watch    # ESP32 respondendo, telemetria chegando
robocar camera          # taxa real da câmera
```

Os três precisam passar. Se `robocar doctor` reclamar de espaço em disco,
resolva **antes** — descobrir que o disco encheu depois de 40 minutos de
gravação é perder 40 minutos.

### Teste do failsafe (obrigatório, toda vez que mexer no firmware)

Carro no cavalete, rodas no ar, armado, motor girando devagar. **Desconecte o
cabo USB.** O motor tem que parar em menos de 300 ms. Se não parar, não coloque
o carro no chão.

---

## Gravando

```bash
# Padrão: pilotando pelo rádio RC (melhor qualidade de dado)
robocar record --control rc --track minicidade_fatec --direction horario

# Sem rádio? Gamepad:
robocar record --control gamepad --track garagem
```

O comando pergunta os metadados, cria a sessão e começa a gravar. `Ctrl+C`
encerra corretamente — drena a fila de escrita e fecha o `session.json`.
**Não mate o processo com `kill -9`**: você perde os últimos frames e os
contadores da sessão.

Durante a gravação, o terminal mostra:

```
frames=  1240  fila= 12  steer=+0.31 thr=+0.28   20.1 Hz  média= 49.8 ms  p95= 52.1 ms  máx= 81.0 ms
```

O que observar:

| Sinal | Significa |
|---|---|
| `fila` subindo sem parar | O disco não acompanha. Reduza a qualidade JPEG ou grave em NVMe |
| Taxa abaixo de ~18 Hz | Câmera, CPU ou disco engasgando |
| `máx` acima de 200 ms | Alguma travada — vale investigar |

Ao final, o resumo mostra o **aproveitamento**. Abaixo de 50%, veja qual
contador subiu:

| Contador | Causa | O que fazer |
|---|---|---|
| `parado` | Carro parado demais | Normal em manobra; se for a maioria, dirija mais contínuo |
| `telemetria_velha` | Enlace serial ruim | Cabo, ruído do motor, taxa do ESP32 |
| `inseguro` | Failsafe / obstáculo / bateria | Ver `robocar link --watch` |
| `descartados_fila` | Disco lento | NVMe, ou qualidade JPEG menor |

---

## O que coletar — plano para chegar competitivo

O erro clássico é gravar 20 voltas idênticas, no mesmo sentido, com a mesma
luz, e concluir que "o modelo não aprende". Ele aprendeu — aprendeu aquela
volta específica.

### Meta mínima

| Condição | Tempo | Por quê |
|---|---|---|
| Sentido horário | 40 min | Base |
| Sentido anti-horário | 40 min | Sem isso, a rede aprende a virar sempre para o mesmo lado |
| Recuperação (ver abaixo) | 20 min | **O mais importante** |
| Iluminação diferente | 30 min | O ginásio não terá a luz da sua garagem |
| Velocidades variadas | 20 min | Generaliza a dinâmica |
| **Total** | **~2h30** | Mínimo para um modelo que dá a volta |

Alvo confortável: **4 a 6 horas** de dado bom, coletadas em pelo menos **4 dias
diferentes**. Dias diferentes trazem luz, poeira e posicionamento de pista
ligeiramente diferentes — que é exatamente a variação que faz o modelo
generalizar.

### Dados de recuperação — o segredo do behavior cloning

Se você só grava condução perfeita, o modelo nunca viu o carro fora do centro
da faixa. Aí, na prova, uma pequena imprecisão o tira do centro, ele encontra
uma cena que nunca viu, erra mais, e o erro cresce até sair da pista. Isso se
chama *distribution shift* e é a causa número 1 de fracasso desses modelos.

**Como coletar recuperação:**

1. Posicione o carro **deslocado** de propósito — encostado na borda direita,
   apontado para fora, atravessado.
2. **Comece a gravar só depois** de já estar corrigindo de volta ao centro.
3. **Pare de gravar** antes de deslocar o carro de novo.

Ou seja: grave **só a correção**, nunca o erro sendo cometido. Se você gravar o
movimento de sair do centro, estará ensinando literalmente a sair da pista.

Na prática, é mais fácil fazer isso em sessões curtas e separadas:

```bash
robocar record --control rc --track minicidade_fatec \
    --tag recuperacao --notes "correcoes da borda direita para o centro"
```

### Variação que vale a pena

- **Sentidos:** horário e anti-horário.
- **Iluminação:** manhã, tarde, noite com luz artificial, cortina aberta/fechada.
- **Velocidade:** lento, médio, no ritmo de prova.
- **Posição de largada:** vários pontos da pista, não só a largada oficial.
- **Obstáculos visuais:** com e sem maquetes, com pessoas em volta (no dia da
  prova haverá público em volta da pista — isso muda o fundo da imagem).

### Variação que NÃO vale a pena

- Pilotagem ruim "para o modelo aprender a corrigir". Ele aprende a pilotar
  mal. Recuperação se coleta do jeito descrito acima.
- Bateria fraca. Com a tensão caindo, o mesmo comando produz velocidade
  diferente e o rótulo fica inconsistente. Troque a bateria antes de gravar.
- Câmera em posição diferente. Se mexeu na câmera, **os dados anteriores
  perdem validade**.

---

## Dia 04/12/2026 — treino na pista oficial

Das 14h às 22h. É a **única** chance de coletar dado da pista real. Plano
sugerido:

| Horário | Atividade |
|---|---|
| 14h–15h | Inspeção, montagem, `robocar doctor`, teste de failsafe |
| 15h–16h | Calibração da direção e da câmera na pista real |
| 16h–18h | **Coleta pesada** — os dois sentidos, várias velocidades |
| 18h–19h | Coleta de recuperação + fotos das placas (Master) |
| 19h–21h | Retreino com os dados do dia + testes autônomos |
| 21h–22h | Ajuste fino, coleta do que faltou, backup |

**Leve o notebook de treino e um HD/SSD externo.** Retreinar com os dados da
pista real na mesma noite é o que separa "funcionou na garagem" de "funcionou
na prova".

**Faça backup a cada sessão.** Um cartão corrompido às 21h apaga o dia inteiro.

---

## Coleta específica do MASTER: placas de trânsito

Além da condução, é preciso um dataset de placas. Elas têm 150 mm, ficam a
400 mm do chão, sempre à direita, 400 mm antes do cruzamento.

Colete fotos de cada tipo de placa:

- de **várias distâncias** (0,5 m a 4 m);
- de **vários ângulos** (a placa aparece de lado ao se aproximar);
- com **iluminação variada**;
- com **fundo variado** (maquetes, público, parede).

Meta: **200+ imagens por classe**. Parece muito, mas 20 minutos de filmagem
com o carro andando devagar já rendem isso — a extração de quadros de vídeo é
o caminho mais rápido.

> **Antes de fechar a lista de classes**, assista ao vídeo oficial das placas:
> <https://youtu.be/YAg2p1kShZA>. A rota do dia é definida por essas placas e
> uma classe faltante significa uma curva errada.

E, para o semáforo (Júnior e Master): grave os três estados, de várias
distâncias, a partir da posição de largada (o semáforo fica a 100 cm dela).
Vídeo oficial: <https://www.youtube.com/watch?v=GvytKrULBW0>

---

## Inspecionando o que foi coletado

```bash
robocar dataset stats --histogram   # resumo + distribuição do esterço
robocar dataset verify              # imagens ausentes, índices duplicados
```

O histograma é a checagem mais reveladora. Se mais da metade das amostras
estiver na faixa central, o modelo vai aprender a andar reto. Isso se resolve
de duas formas, e vale fazer as duas:

1. **Balanceamento no treino** (`--balance`).
2. **Coletar mais curva** — mais voltas em trechos sinuosos, menos reta.

## Marcando sessões ruins

Bateu, saiu da pista, alguém entrou na frente? Não apague — marque:

```bash
robocar dataset tag 20261204_163045_minicidade_fatec \
    --discard "bateu no quarteirao aos 3min"
```

A sessão continua no disco (útil para entender o que deu errado) mas o loader
de treino a ignora. `--keep` desfaz.
