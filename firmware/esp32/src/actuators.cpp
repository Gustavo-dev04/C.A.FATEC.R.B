#include "actuators.h"

#include "protocol.h"

void Actuators::begin(const Settings& settings) {
  // O ESP32Servo precisa de timers de hardware alocados explicitamente.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  steeringServo_.setPeriodHertz(PWM_FREQ_HZ);
  steeringServo_.attach(PIN_STEERING, PULSE_MIN_US, PULSE_MAX_US);
  throttleEsc_.setPeriodHertz(PWM_FREQ_HZ);
  throttleEsc_.attach(PIN_THROTTLE, PULSE_MIN_US, PULSE_MAX_US);

  // Estado inicial seguro: rodas retas, motor em neutro.
  steer_ = 0.0f;
  throttle_ = 0.0f;
  steeringServo_.writeMicroseconds(settings.steerCenterUs);
  throttleEsc_.writeMicroseconds(settings.thrNeutralUs);

  // Muitos ESC exigem ver o neutro por ~2 s antes de armar. Sem isso, alguns
  // entram em modo de programação em vez de funcionar.
  delay(2000);
}

uint16_t Actuators::steerToPulse(const Settings& settings, float steer) const {
  float value = clampf(steer, -1.0f, 1.0f);
  if (settings.steerInvert) value = -value;

  // Interpolação por lado: centro e batentes raramente são simétricos numa
  // direção Ackermann real, e forçar simetria custa esterço útil.
  const float pulse =
      value >= 0.0f
          ? settings.steerCenterUs +
                value * (settings.steerMaxUs - settings.steerCenterUs)
          : settings.steerCenterUs +
                value * (settings.steerCenterUs - settings.steerMinUs);

  return static_cast<uint16_t>(clampf(pulse, PULSE_MIN_US, PULSE_MAX_US));
}

uint16_t Actuators::throttleToPulse(const Settings& settings,
                                    float throttle) const {
  float value = clampf(throttle, -1.0f, 1.0f);

  if (fabsf(value) < settings.thrDeadband) {
    return settings.thrNeutralUs;
  }

  const float pulse =
      value >= 0.0f
          ? settings.thrNeutralUs +
                value * (settings.thrMaxUs - settings.thrNeutralUs)
          : settings.thrNeutralUs +
                value * (settings.thrNeutralUs - settings.thrMinUs);

  return static_cast<uint16_t>(clampf(pulse, PULSE_MIN_US, PULSE_MAX_US));
}

void Actuators::apply(const Settings& settings, float steer, float throttle,
                      bool armed, float dt) {
  // 1. Tetos de segurança. `min` com a constante de compilação garante que o
  //    Jetson só consegue APERTAR o limite, nunca afrouxar.
  const float limitFwd = min(settings.thrLimitFwd, HARD_LIMIT_FORWARD);
  const float limitRev = min(settings.thrLimitRev, HARD_LIMIT_REVERSE);
  float target = clampf(throttle, -limitRev, limitFwd);

  // 2. Desarmado significa motor em neutro, sem exceção.
  if (!armed) target = 0.0f;

  // 3. Rampa: protege a transmissão e evita que a roda perca aderência.
  steer_ = applySlew(steer_, clampf(steer, -1.0f, 1.0f), settings.slewSteer, dt);
  throttle_ = applySlew(throttle_, target, settings.slewThr, dt);

  steeringServo_.writeMicroseconds(steerToPulse(settings, steer_));
  throttleEsc_.writeMicroseconds(throttleToPulse(settings, throttle_));
}

void Actuators::emergencyStop(const Settings& settings) {
  throttle_ = 0.0f;  // sem rampa: é uma parada de emergência
  throttleEsc_.writeMicroseconds(settings.thrNeutralUs);
  // steer_ permanece como está — ver comentário no cabeçalho.
  steeringServo_.writeMicroseconds(steerToPulse(settings, steer_));
}
