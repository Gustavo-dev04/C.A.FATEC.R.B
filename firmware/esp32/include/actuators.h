/**
 * Direção e tração: rampa, limites e conversão para pulso de PWM.
 *
 * Toda saída para o servo e para o ESC passa por aqui. Concentrar isso em um
 * lugar é o que permite garantir, olhando pouco código, que nenhum comando
 * absurdo chega aos atuadores.
 */
#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>

#include "config.h"

class Actuators {
 public:
  void begin(const Settings& settings);

  /**
   * Aplica um comando normalizado, com rampa e limites.
   *
   * @param steer     [-1, +1]; negativo = esquerda
   * @param throttle  [-1, +1]; negativo = ré
   * @param armed     desarmado força a tração em neutro
   * @param dt        segundos desde a última chamada (para a rampa)
   */
  void apply(const Settings& settings, float steer, float throttle, bool armed,
             float dt);

  /**
   * Neutro imediato de tração, SEM rampa.
   *
   * A direção mantém o último valor de propósito: travar as rodas no centro a
   * 2 m/s joga o carro para fora da trajetória, enquanto manter o esterço
   * deixa ele desacelerar na curva em que já estava. Ver docs/03.
   */
  void emergencyStop(const Settings& settings);

  float appliedSteer() const { return steer_; }
  float appliedThrottle() const { return throttle_; }

 private:
  uint16_t steerToPulse(const Settings& settings, float steer) const;
  uint16_t throttleToPulse(const Settings& settings, float throttle) const;

  Servo steeringServo_;
  Servo throttleEsc_;
  float steer_ = 0.0f;
  float throttle_ = 0.0f;
};

/// Aproxima `current` de `target` respeitando `ratePerSec`.
inline float applySlew(float current, float target, float ratePerSec, float dt) {
  if (ratePerSec <= 0.0f || dt <= 0.0f) return target;
  const float maxDelta = ratePerSec * dt;
  const float delta = target - current;
  if (delta > maxDelta) return current + maxDelta;
  if (delta < -maxDelta) return current - maxDelta;
  return target;
}
