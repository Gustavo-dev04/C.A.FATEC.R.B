# Checklist de competição — 04 e 05/12/2026

Imprima e leve em papel. No dia, ninguém vai abrir o repositório para conferir
se lembrou de desligar o Wi-Fi.

---

## Semanas antes

- [ ] Inscrição feita no site (**não é possível trocar de categoria depois**)
- [ ] Categoria decidida: **MASTER** (ou JUNIOR)
- [ ] Termo de menor de idade enviado, se aplicável, até **3 dias antes**
      (robocar.race@gmail.com)
- [ ] Dúvidas do regulamento enviadas por e-mail
      ([lista em docs/00 §9](00-regulamento-resumo.md#9-pontos-a-confirmar-com-a-organização))
- [ ] Carro medido: **≤ 500 × 250 mm** — com a câmera e todos os suportes montados
- [ ] `robocar doctor` passando
- [ ] Teste de failsafe feito e refeito
- [ ] Pelo menos 2h30 de dataset coletado em condições variadas
- [ ] Modelo treinado com MAE < 0.08 e testado na pista de treino

---

## Bagagem

### Essencial

- [ ] Carro
- [ ] Jetson + cartão/SSD com o sistema **e um clone de backup**
- [ ] Notebook com o repositório e o ambiente de treino funcionando
- [ ] Baterias LiPo (**pelo menos 3**) + carregador + bolsa de segurança
- [ ] Power bank do Jetson, carregado
- [ ] Cabos USB (leve 3 — o "cabo que só carrega" é um clássico)
- [ ] **Adaptador de tomada 220 V sem pino terra** e extensão
- [ ] Rádio RC + pilhas
- [ ] HD/SSD externo para backup do dataset do dia

### Peças de reposição

> ⚠️ **Trocar o robô ou substituir componentes embarcados de forma substancial
> = desclassificação.** Ajustes e pequenos consertos são permitidos. Na
> dúvida, pergunte ao juiz **antes** de trocar qualquer coisa.

- [ ] Servo de direção sobressalente
- [ ] Pneus e rodas
- [ ] Parafusos, porcas, abraçadeiras, fita dupla-face
- [ ] Fios, jumpers, ferro de solda + solda
- [ ] Chaves Allen e de fenda

### Não esquecer

- [ ] Água e comida (**a organização não fornece**)
- [ ] Este checklist impresso
- [ ] Multímetro
- [ ] Fita crepe e caneta (marcar posições de calibração)

---

## Sexta 04/12 — treino e testes (14h–22h)

### Chegada (14h–15h)

- [ ] Inspeção prévia com a organização
- [ ] Montar o box, ligar a bancada
- [ ] Carregar todas as baterias
- [ ] **Fotografar e medir a pista real** — largura da rua, cor exata do
      carpete, altura e tipo das placas
- [ ] Conferir a iluminação do local (define a exposição da câmera)

### Preparação (15h–16h)

- [ ] `robocar doctor`
- [ ] `robocar link --watch` — telemetria, tensão de bateria, ultrassônicos
- [ ] **Teste de failsafe** com o carro no cavalete
- [ ] `robocar calib steering` na pista real
- [ ] `robocar camera --snapshot` — ajustar exposição para a luz do ginásio
- [ ] **`robocar camera --focus`** — conferir a nitidez contra a referência
      anotada em `config/camera.yaml`. A lente tem foco por rosca: se levou
      esbarrão na viagem, desfocou
- [ ] Rosca de foco **travada** e traço de caneta alinhado
- [ ] Confirmar que a captura está em **1640×1232** (campo completo), não em
      1280×720 (recortado)
- [ ] **Marcar a posição da câmera com fita** e não mexer mais

### Coleta (16h–19h) — a parte mais valiosa do dia

- [ ] 45 min sentido horário
- [ ] 45 min sentido anti-horário
- [ ] 30 min de **recuperação** (gravando só a correção — ver docs/04)
- [ ] 20 min em velocidades variadas
- [ ] **MASTER:** fotos de todas as placas, de 0,5 a 4 m, vários ângulos
- [ ] Semáforo: gravar os 3 estados a partir da largada
- [ ] `robocar dataset stats --histogram` a cada 2 sessões
- [ ] **Backup no HD externo a cada sessão**

### Treino e validação (19h–21h)

- [ ] Retreinar com os dados do dia: `python -m robocar_ml.train --balance`
- [ ] Exportar ONNX e gerar o engine TensorRT **no Jetson**
- [ ] Teste no cavalete: o servo responde no sentido certo?
- [ ] Volta autônoma com `throttle.limit_auto = 0.25`
- [ ] Subir a velocidade um degrau por vez, uma volta limpa por degrau

### Antes de sair (21h–22h)

- [ ] Coletar o que faltou
- [ ] **Backup final** do dataset e dos modelos
- [ ] Carregar todas as baterias para o dia seguinte
- [ ] Deixar o carro montado e testado — **não desmonte nada**

---

## Sábado 05/12 — prova (a partir das 9h)

### Antes da chamada

- [ ] Baterias carregadas e conferidas com multímetro
- [ ] `robocar doctor`
- [ ] Teste de failsafe (sim, de novo)
- [ ] Câmera na posição marcada com fita, parafusos apertados
- [ ] Modelo correto carregado — confira a data do arquivo
- [ ] Sorteio da ordem de largada anotado

### 🚨 Perfil de prova — obrigatório

```bash
robocar drive --profile race
```

Confira, **um por um**:

- [ ] **Wi-Fi desligado** (`nmcli radio all off`)
- [ ] **Bluetooth desligado**
- [ ] **Rádio RC desligado** (`rc_enable = 0`)
- [ ] **Nenhum notebook conectado ao carro**
- [ ] O carro liga, carrega o modelo e arma **sem SSH**
- [ ] LED de status visível e no padrão certo

> "O sensoriamento deve ser 100% embarcado — não é permitida comunicação com
> computação ou sensoriamento externo." Descumprir é desclassificação.

### Na largada

- [ ] Carro posicionado, armado pelo botão físico
- [ ] LED amarelo (armado, aguardando)
- [ ] **Não se mover antes do verde** — largada antecipada custa +20 s
      (ou −2 m se não completar)
- [ ] Detector de semáforo confirmado funcionando no local exato da largada

### Durante

- [ ] **Ninguém toca no carro** durante a volta
- [ ] Manter-se na **mão direita** — cada invasão/saída custa 10 s
- [ ] Anotar tempo e erros de cada volta
- [ ] 3 voltas cronometradas; **vale a melhor**

### Entre as voltas

- [ ] Trocar a bateria (permitido)
- [ ] Ajustes finos permitidos — **troca substancial de componente, não**
- [ ] Revisar o log da volta anterior

---

## Se der errado

| Problema | Primeira ação |
|---|---|
| Carro não anda | LED de status. `CFG_PENDING`? `FAILSAFE`? |
| Vai reto e sai da pista | Modelo aprendeu "reta". Reduza a velocidade; se der tempo, retreine com `--balance` |
| Trepida na direção | Alimentação do servo (use o BEC, não o USB) |
| Reinicia no meio | Baterias compartilhadas — separe Jetson e tração |
| Sem telemetria | Cabo USB. Teste com outro |
| Freia sozinho | Ultrassônico com eco fantasma; suba `obstacle_stop_mm` |
| Bate no quarteirão | Coletar recuperação naquele trecho específico |

**Regra de ouro do dia:** o critério de desempate é **menor número de erros**.
Se estiver na dúvida entre velocidade e estabilidade, escolha estabilidade —
um carro lento que completa vence um carro rápido que sai da pista.

---

## Depois

- [ ] Backup de tudo antes de desmontar
- [ ] Anotar o que funcionou e o que não funcionou (vira ADR)
- [ ] Trocar contato com as outras equipes — o regulamento diz que o objetivo
      principal é aprendizado e networking, e ele tem razão
