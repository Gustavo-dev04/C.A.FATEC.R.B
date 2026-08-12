#include "ultrasonic.h"

namespace {

// Estado compartilhado com as ISRs. `volatile` porque o loop principal lê o
// que a interrupção escreve.
volatile uint32_t echoRiseUs[NUM_DISTANCE_SENSORS] = {0};
volatile uint32_t echoDurationUs[NUM_DISTANCE_SENSORS] = {0};
volatile bool echoComplete[NUM_DISTANCE_SENSORS] = {false};

// Uma ISR por pino. Mede a largura do pulso de eco: sobe = início,
// desce = fim. IRAM_ATTR mantém o código na RAM interna, obrigatório no ESP32
// para ISRs que podem disparar durante acesso à flash.
template <uint8_t Index>
void IRAM_ATTR onEchoChange() {
  if (digitalRead(PIN_ECHO[Index]) == HIGH) {
    echoRiseUs[Index] = micros();
  } else if (echoRiseUs[Index] != 0) {
    echoDurationUs[Index] = micros() - echoRiseUs[Index];
    echoComplete[Index] = true;
  }
}

}  // namespace

void UltrasonicArray::begin() {
  for (uint8_t i = 0; i < NUM_DISTANCE_SENSORS; ++i) {
    pinMode(PIN_TRIG[i], OUTPUT);
    digitalWrite(PIN_TRIG[i], LOW);
    pinMode(PIN_ECHO[i], INPUT);
    distances_[i] = 0;
  }
  // Os índices precisam ser constantes de compilação para o template.
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO[0]), onEchoChange<0>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO[1]), onEchoChange<1>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO[2]), onEchoChange<2>, CHANGE);

  state_ = State::Idle;
  current_ = 0;
  stateSinceUs_ = micros();
}

void UltrasonicArray::startMeasurement(uint8_t index) {
  noInterrupts();
  echoComplete[index] = false;
  echoRiseUs[index] = 0;
  echoDurationUs[index] = 0;
  interrupts();

  // Pulso de disparo do HC-SR04: 10 us em nível alto.
  digitalWrite(PIN_TRIG[index], LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG[index], HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG[index], LOW);
}

void UltrasonicArray::finishMeasurement(uint8_t index, uint32_t durationUs) {
  const float mm = durationUs * US_TO_MM;
  if (mm > 0.0f && mm <= DISTANCE_MAX_MM) {
    distances_[index] = static_cast<uint16_t>(mm);
    lastValidMs_[index] = millis();
  } else {
    // Fora de alcance vira 0 ("sem leitura"), não uma distância enorme:
    // o consumidor precisa distinguir "longe" de "não sei".
    distances_[index] = 0;
  }
}

void UltrasonicArray::update() {
  const uint32_t now = micros();

  switch (state_) {
    case State::Idle:
      if (now - stateSinceUs_ >= SENSOR_SETTLE_US) {
        startMeasurement(current_);
        stateSinceUs_ = now;
        state_ = State::Waiting;
      }
      break;

    case State::Waiting: {
      bool complete;
      uint32_t duration;
      noInterrupts();
      complete = echoComplete[current_];
      duration = echoDurationUs[current_];
      interrupts();

      if (complete) {
        finishMeasurement(current_, duration);
      } else if (now - stateSinceUs_ >= ECHO_TIMEOUT_US) {
        // Sem eco: superfície absorvente, ângulo agudo ou nada à frente.
        distances_[current_] = 0;
      } else {
        break;  // ainda esperando
      }

      current_ = (current_ + 1) % NUM_DISTANCE_SENSORS;
      stateSinceUs_ = now;
      state_ = State::Idle;
      break;
    }

    case State::Triggering:
      state_ = State::Idle;
      break;
  }
}

uint16_t UltrasonicArray::frontDistance() const {
  uint16_t best = DISTANCE_MAX_MM;
  for (uint8_t i = 0; i < NUM_DISTANCE_SENSORS; ++i) {
    if (distances_[i] != 0 && distances_[i] < best) {
      best = distances_[i];
    }
  }
  return best;
}

uint32_t UltrasonicArray::ageMs(uint8_t index) const {
  if (index >= NUM_DISTANCE_SENSORS || lastValidMs_[index] == 0) {
    return UINT32_MAX;
  }
  return millis() - lastValidMs_[index];
}
