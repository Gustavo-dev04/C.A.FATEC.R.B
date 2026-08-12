/**
 * C.A.FATEC.R.B — firmware do ESP32 (RoboCar Race 2026)
 *
 * Responsabilidades, e só estas:
 *   - receber $CMD do Jetson e aplicar nos atuadores, com rampa e limites;
 *   - FAILSAFE: sem comando por `timeoutMs`, motor em neutro;
 *   - ler os sensores de distância e cortar a tração diante de obstáculo;
 *   - medir a tensão da bateria;
 *   - publicar $TLM a 50 Hz — inclusive o valor APLICADO, que é o rótulo do
 *     dataset (ver docs/adr/0002);
 *   - opcionalmente, ler o rádio RC durante a coleta de dados.
 *
 * Nada de visão, nada de decisão de rota: isso é do Jetson. O que este
 * firmware precisa ser é curto o bastante para ser lido inteiro e confiável o
 * bastante para continuar valendo quando o Jetson travar.
 *
 * Protocolo: docs/03-protocolo-serial.md
 */

#include <Arduino.h>

#include "actuators.h"
#include "config.h"
#include "protocol.h"
#include "ultrasonic.h"

namespace {

Settings settings;
Actuators actuators;
UltrasonicArray sensors;
LineReader reader;

// --- estado ---------------------------------------------------------------
uint16_t lastSeq = 0;
uint32_t lastCommandMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastBatteryMs = 0;
uint32_t lastLoopUs = 0;
uint32_t lastLedMs = 0;

float commandSteer = 0.0f;
float commandThrottle = 0.0f;
bool armed = false;
bool configReceived = false;
bool failsafe = true;  // começa em failsafe: só sai quando o Jetson falar
float batteryVolts = 0.0f;
uint8_t configCount = 0;

// --- botão de armar -------------------------------------------------------
bool lastButtonState = HIGH;
uint32_t lastButtonMs = 0;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 50;

#if RC_INPUT_ENABLED
// --- entrada de rádio RC (só treino/teste) --------------------------------
volatile uint32_t rcRiseUs[2] = {0, 0};
volatile uint16_t rcPulseUs[2] = {0, 0};
volatile uint32_t rcLastUs[2] = {0, 0};
constexpr uint32_t RC_TIMEOUT_US = 100000;  // 100 ms sem pulso = rádio mudo
constexpr uint16_t RC_MIN_US = 1000;
constexpr uint16_t RC_MAX_US = 2000;
constexpr uint16_t RC_CENTER_US = 1500;
constexpr uint16_t RC_DEADBAND_US = 30;

template <uint8_t Channel>
void IRAM_ATTR onRcChange() {
  const uint8_t pin = Channel == 0 ? PIN_RC_STEER : PIN_RC_THROTTLE;
  if (digitalRead(pin) == HIGH) {
    rcRiseUs[Channel] = micros();
  } else if (rcRiseUs[Channel] != 0) {
    const uint32_t width = micros() - rcRiseUs[Channel];
    // Ignora larguras impossíveis: ruído do motor gera pulsos espúrios.
    if (width >= 800 && width <= 2200) {
      rcPulseUs[Channel] = static_cast<uint16_t>(width);
      rcLastUs[Channel] = micros();
    }
  }
}

float rcNormalized(uint8_t channel) {
  uint16_t pulse;
  noInterrupts();
  pulse = rcPulseUs[channel];
  interrupts();
  if (pulse == 0) return 0.0f;

  const int delta = static_cast<int>(pulse) - RC_CENTER_US;
  if (abs(delta) < RC_DEADBAND_US) return 0.0f;
  const float span =
      delta > 0 ? (RC_MAX_US - RC_CENTER_US) : (RC_CENTER_US - RC_MIN_US);
  return clampf(delta / span, -1.0f, 1.0f);
}

bool rcValid() {
  const uint32_t now = micros();
  noInterrupts();
  const uint32_t steerAge = now - rcLastUs[0];
  const uint32_t throttleAge = now - rcLastUs[1];
  const bool everSeen = rcLastUs[0] != 0 && rcLastUs[1] != 0;
  interrupts();
  return everSeen && steerAge < RC_TIMEOUT_US && throttleAge < RC_TIMEOUT_US;
}
#endif  // RC_INPUT_ENABLED

// --- configuração ---------------------------------------------------------

/// Aplica uma chave de $CFG. @return false se a chave for desconhecida.
bool applyConfig(const char* key, const char* value) {
  const float number = atof(value);
  const long integer = atol(value);

  if (!strcmp(key, "steer_center_us")) {
    settings.steerCenterUs = constrain(integer, PULSE_MIN_US, PULSE_MAX_US);
  } else if (!strcmp(key, "steer_min_us")) {
    settings.steerMinUs = constrain(integer, PULSE_MIN_US, PULSE_MAX_US);
  } else if (!strcmp(key, "steer_max_us")) {
    settings.steerMaxUs = constrain(integer, PULSE_MIN_US, PULSE_MAX_US);
  } else if (!strcmp(key, "steer_invert")) {
    settings.steerInvert = integer != 0;
  } else if (!strcmp(key, "thr_neutral_us")) {
    settings.thrNeutralUs = constrain(integer, PULSE_MIN_US, PULSE_MAX_US);
  } else if (!strcmp(key, "thr_min_us")) {
    settings.thrMinUs = constrain(integer, PULSE_MIN_US, PULSE_MAX_US);
  } else if (!strcmp(key, "thr_max_us")) {
    settings.thrMaxUs = constrain(integer, PULSE_MIN_US, PULSE_MAX_US);
  } else if (!strcmp(key, "thr_deadband")) {
    settings.thrDeadband = clampf(number, 0.0f, 0.5f);
  } else if (!strcmp(key, "thr_limit_fwd")) {
    // Limites de segurança só apertam. Ver docs/03-protocolo-serial.md.
    settings.thrLimitFwd = min(clampf(number, 0.0f, 1.0f), HARD_LIMIT_FORWARD);
  } else if (!strcmp(key, "thr_limit_rev")) {
    settings.thrLimitRev = min(clampf(number, 0.0f, 1.0f), HARD_LIMIT_REVERSE);
  } else if (!strcmp(key, "slew_steer")) {
    settings.slewSteer = clampf(number, 0.1f, 100.0f);
  } else if (!strcmp(key, "slew_thr")) {
    settings.slewThr = clampf(number, 0.1f, 100.0f);
  } else if (!strcmp(key, "timeout_ms")) {
    settings.timeoutMs = min<uint32_t>(max<long>(integer, 50), HARD_TIMEOUT_MS);
  } else if (!strcmp(key, "obst_stop_mm")) {
    settings.obstStopMm = constrain(integer, 0, DISTANCE_MAX_MM);
  } else if (!strcmp(key, "obst_slow_mm")) {
    settings.obstSlowMm = constrain(integer, 0, DISTANCE_MAX_MM);
  } else if (!strcmp(key, "tlm_hz")) {
    settings.tlmHz = constrain(integer, 1, 200);
  } else if (!strcmp(key, "rc_enable")) {
    settings.rcEnable = integer != 0;
  } else {
    return false;
  }

  ++configCount;
  configReceived = true;
  return true;
}

// --- tratamento de quadros ------------------------------------------------

void handleFrame(Frame& frame) {
  const char* type = frame.type;

  if (!strcmp(type, "CMD") && frame.fieldCount == 4) {
    lastSeq = static_cast<uint16_t>(atol(frame.fields[0]));
    commandSteer = clampf(atof(frame.fields[1]), -1.0f, 1.0f);
    commandThrottle = clampf(atof(frame.fields[2]), -1.0f, 1.0f);
    lastCommandMs = millis();
    if (failsafe) {
      failsafe = false;
      sendLog('I', "failsafe liberado");
    }
    return;
  }

  if (!strcmp(type, "ARM") && frame.fieldCount == 1) {
    armed = atol(frame.fields[0]) != 0;
    sendAck("ARM", frame.fields[0]);
    return;
  }

  if (!strcmp(type, "CFG") && frame.fieldCount == 2) {
    if (applyConfig(frame.fields[0], frame.fields[1])) {
      sendAck("CFG", frame.fields[0]);
    } else {
      sendErr(ErrCode::UnknownConfigKey, frame.fields[0]);
    }
    return;
  }

  if (!strcmp(type, "PING") && frame.fieldCount == 1) {
    sendPong(static_cast<uint16_t>(atol(frame.fields[0])), millis());
    return;
  }

  sendErr(ErrCode::UnknownType, type);
}

// --- periféricos ----------------------------------------------------------

void readBattery() {
  // analogReadMilliVolts aplica a calibração de fábrica do ADC; analogRead()
  // cru no ESP32 é bem não-linear e daria uma leitura de tensão inútil.
  const uint32_t millivolts = analogReadMilliVolts(PIN_BATTERY);
  const float measured = millivolts / 1000.0f * settings.batteryDivider;
  // Filtro passa-baixa: o motor faz a tensão oscilar muito a cada arranque.
  batteryVolts = batteryVolts == 0.0f ? measured
                                      : batteryVolts * 0.9f + measured * 0.1f;
}

void updateArmButton() {
  const bool state = digitalRead(PIN_ARM_BUTTON);
  const uint32_t now = millis();
  if (state != lastButtonState && now - lastButtonMs > BUTTON_DEBOUNCE_MS) {
    lastButtonMs = now;
    lastButtonState = state;
    if (state == LOW) {  // pressionado (INPUT_PULLUP)
      armed = !armed;
      sendLog('I', armed ? "armado pelo botao" : "desarmado pelo botao");
    }
  }
}

void updateLed(LedPattern pattern) {
  const uint32_t now = millis();
  if (now - lastLedMs < LED_PERIOD_MS) return;
  lastLedMs = now;

  // Cada padrão é uma máscara de 20 posições (1 s a 50 ms por posição).
  static uint8_t phase = 0;
  phase = (phase + 1) % 20;

  bool on = false;
  switch (pattern) {
    case LedPattern::WaitingConfig:   on = phase < 2; break;             // 1 Hz
    case LedPattern::Ready:           on = phase < 2 || (phase >= 10 && phase < 12); break;
    case LedPattern::Armed:           on = true; break;                  // fixo
    case LedPattern::Failsafe:        on = (phase % 4) < 2; break;       // rápido
    case LedPattern::BatteryCritical: on = (phase % 7) < 2; break;       // 3 Hz
  }
  digitalWrite(PIN_LED, on ? HIGH : LOW);
}

uint8_t buildFlags(bool obstacleStop, bool obstacleSlow, bool rcActive) {
  uint8_t flags = 0;
  if (armed) flags |= Flag::Armed;
  if (failsafe) flags |= Flag::Failsafe;
  if (rcActive) flags |= Flag::RcValid;
  if (obstacleStop) flags |= Flag::ObstacleStop;
  if (obstacleSlow) flags |= Flag::ObstacleSlow;
  if (batteryVolts > 0.5f && batteryVolts < settings.batteryMinV) {
    flags |= Flag::BattLow;
  }
  if (batteryVolts > 0.5f && batteryVolts < settings.batteryCriticalV) {
    flags |= Flag::BattCritical;
  }
  if (!configReceived) flags |= Flag::CfgPending;
  return flags;
}

}  // namespace

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  pinMode(PIN_ARM_BUTTON, INPUT_PULLUP);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_BATTERY, ADC_11db);  // faixa até ~3,3 V

  sensors.begin();
  actuators.begin(settings);  // inclui os 2 s de neutro exigidos pelo ESC

#if RC_INPUT_ENABLED
  pinMode(PIN_RC_STEER, INPUT);
  pinMode(PIN_RC_THROTTLE, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_STEER), onRcChange<0>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_THROTTLE), onRcChange<1>, CHANGE);
#endif

  lastLoopUs = micros();
  lastCommandMs = millis();
  sendLog('I', "robocar esp32 pronto");
}

void loop() {
  const uint32_t nowMs = millis();
  const uint32_t nowUs = micros();
  const float dt = (nowUs - lastLoopUs) / 1e6f;
  lastLoopUs = nowUs;

  // 1. Entrada serial -------------------------------------------------------
  char* line = reader.poll(Serial);
  if (line != nullptr) {
    Frame frame;
    uint8_t err = 0;
    if (parseFrame(line, frame, err)) {
      handleFrame(frame);
    } else {
      sendErr(err, "");
    }
  }

  // 2. Sensores -------------------------------------------------------------
  sensors.update();
  updateArmButton();

  if (nowMs - lastBatteryMs >= BATTERY_PERIOD_MS) {
    lastBatteryMs = nowMs;
    readBattery();
  }

  // 3. Failsafe -------------------------------------------------------------
  // A única coisa que não pode falhar neste firmware.
  if (nowMs - lastCommandMs > settings.timeoutMs) {
    if (!failsafe) {
      failsafe = true;
      sendLog('W', "FAILSAFE: sem comando do jetson");
    }
  }

  // 4. Escolha da fonte de comando -----------------------------------------
  float steer = commandSteer;
  float throttle = commandThrottle;
  Source source = failsafe ? Source::Failsafe : Source::Serial;
  bool rcActive = false;

#if RC_INPUT_ENABLED
  if (settings.rcEnable && rcValid()) {
    // O rádio tem prioridade sobre a serial durante a coleta de dados: quem
    // pilota é o humano, e o $CMD do Jetson serve só de batimento cardíaco.
    steer = rcNormalized(0);
    throttle = rcNormalized(1);
    source = Source::Rc;
    rcActive = true;
  }
#endif

  // 5. Obstáculos -----------------------------------------------------------
  const uint16_t front = sensors.frontDistance();
  const bool obstacleStop = front < settings.obstStopMm;
  const bool obstacleSlow = !obstacleStop && front < settings.obstSlowMm;

  if (obstacleStop && throttle > 0.0f) {
    throttle = 0.0f;  // corta apenas o avanço; a ré continua liberada
  } else if (obstacleSlow && throttle > 0.0f) {
    throttle *= 0.5f;
  }

  // 6. Atuação --------------------------------------------------------------
  const bool batteryCritical =
      batteryVolts > 0.5f && batteryVolts < settings.batteryCriticalV;

  if (failsafe || batteryCritical) {
    actuators.emergencyStop(settings);
  } else {
    actuators.apply(settings, steer, throttle, armed, dt);
  }

  // 7. Telemetria -----------------------------------------------------------
  const uint32_t telemetryPeriodMs = 1000 / max<uint16_t>(settings.tlmHz, 1);
  if (nowMs - lastTelemetryMs >= telemetryPeriodMs) {
    lastTelemetryMs = nowMs;
    sendTelemetry(lastSeq, nowMs, actuators.appliedSteer(),
                  actuators.appliedThrottle(), source, sensors.distances(),
                  NUM_DISTANCE_SENSORS, batteryVolts,
                  buildFlags(obstacleStop, obstacleSlow, rcActive));
  }

  // 8. LED de status --------------------------------------------------------
  LedPattern pattern = LedPattern::Ready;
  if (batteryCritical) {
    pattern = LedPattern::BatteryCritical;
  } else if (!configReceived) {
    pattern = LedPattern::WaitingConfig;
  } else if (failsafe) {
    pattern = LedPattern::Failsafe;
  } else if (armed) {
    pattern = LedPattern::Armed;
  }
  updateLed(pattern);
}
