/**
 * Leitura não bloqueante de sensores ultrassônicos HC-SR04.
 *
 * Dois cuidados que definem o desenho deste módulo:
 *
 * 1. **Nada de pulseIn().** A função bloqueia até 25 ms por sensor. Com três
 *    sensores, o loop de controle cairia para ~13 Hz e o PWM engasgaria.
 *    Aqui o eco é medido por interrupção e o loop nunca espera.
 *
 * 2. **Disparo em rodízio, um sensor por vez.** Sensores disparados juntos
 *    escutam o eco um do outro (crosstalk) e devolvem distâncias fantasma —
 *    justamente o tipo de leitura que faria o carro frear do nada no meio da
 *    prova.
 */
#pragma once

#include <Arduino.h>

#include "config.h"

class UltrasonicArray {
 public:
  /// Configura pinos e interrupções. Chamar uma vez no setup().
  void begin();

  /// Avança a máquina de estados. Chamar a cada loop(); nunca bloqueia.
  void update();

  /// Última distância válida do sensor `index`, em mm. 0 = sem leitura.
  uint16_t distance(uint8_t index) const {
    return index < NUM_DISTANCE_SENSORS ? distances_[index] : 0;
  }

  const uint16_t* distances() const { return distances_; }

  /// Menor distância frontal válida, em mm. `DISTANCE_MAX_MM` se não há eco.
  uint16_t frontDistance() const;

  /// Idade da última leitura válida do sensor, em ms.
  uint32_t ageMs(uint8_t index) const;

 private:
  enum class State : uint8_t { Idle, Triggering, Waiting };

  void startMeasurement(uint8_t index);
  void finishMeasurement(uint8_t index, uint32_t durationUs);

  uint16_t distances_[NUM_DISTANCE_SENSORS] = {0};
  uint32_t lastValidMs_[NUM_DISTANCE_SENSORS] = {0};
  uint8_t current_ = 0;
  State state_ = State::Idle;
  uint32_t stateSinceUs_ = 0;
};
