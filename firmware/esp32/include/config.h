/**
 * Pinos, limites de segurança e valores padrão do firmware.
 *
 * Regra que organiza este arquivo: os TETOS DE SEGURANÇA são constantes de
 * compilação e o Jetson não consegue afrouxá-los. Ele pode APERTAR qualquer
 * limite via $CFG, nunca aumentar. Um bug no Python não deve conseguir soltar
 * o carro além do que este arquivo permite.
 *
 * Mexeu em pino aqui? Atualize a tabela em docs/02-hardware.md.
 */
#pragma once

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Pinos
// ---------------------------------------------------------------------------

constexpr uint8_t PIN_STEERING = 18;  // servo da direção dianteira
constexpr uint8_t PIN_THROTTLE = 19;  // ESC da tração

// Sensores de distância (HC-SR04). ATENÇÃO: o ECHO é 5 V — use divisor
// resistivo 1k/2k em cada um ou o GPIO do ESP32 queima.
constexpr uint8_t NUM_DISTANCE_SENSORS = 3;
constexpr uint8_t PIN_TRIG[NUM_DISTANCE_SENSORS] = {25, 32, 27};
constexpr uint8_t PIN_ECHO[NUM_DISTANCE_SENSORS] = {26, 33, 14};
// Índices: 0 = frontal esquerdo (-35°), 1 = frontal central, 2 = frontal direito (+35°)
constexpr uint8_t SENSOR_FRONT = 1;

constexpr uint8_t PIN_BATTERY = 34;   // ADC1_CH6 — só entrada, sem pull-up
constexpr uint8_t PIN_LED = 2;        // LED embutido da maioria das DevKit
constexpr uint8_t PIN_ARM_BUTTON = 15;  // botão para o GND, com INPUT_PULLUP

#if RC_INPUT_ENABLED
constexpr uint8_t PIN_RC_STEER = 4;
constexpr uint8_t PIN_RC_THROTTLE = 5;
#endif

// ---------------------------------------------------------------------------
// Tetos de segurança — o Jetson NUNCA passa destes valores
// ---------------------------------------------------------------------------

constexpr float HARD_LIMIT_FORWARD = 0.75f;  ///< aceleração máxima absoluta
constexpr float HARD_LIMIT_REVERSE = 0.35f;  ///< ré máxima absoluta
constexpr uint32_t HARD_TIMEOUT_MS = 500;    ///< failsafe nunca além disso
constexpr uint16_t PULSE_MIN_US = 800;       ///< limite físico do servo
constexpr uint16_t PULSE_MAX_US = 2400;

// ---------------------------------------------------------------------------
// Valores padrão (substituídos pelo $CFG vindo de config/vehicle.yaml)
// ---------------------------------------------------------------------------

struct Settings {
  // Direção
  uint16_t steerCenterUs = 1500;
  uint16_t steerMinUs = 1150;
  uint16_t steerMaxUs = 1850;
  bool steerInvert = false;
  float slewSteer = 6.0f;

  // Tração
  uint16_t thrNeutralUs = 1500;
  uint16_t thrMinUs = 1000;
  uint16_t thrMaxUs = 2000;
  float thrDeadband = 0.05f;
  float thrLimitFwd = 0.30f;
  float thrLimitRev = 0.20f;
  float slewThr = 2.0f;

  // Segurança
  uint32_t timeoutMs = 250;
  uint16_t obstStopMm = 250;
  uint16_t obstSlowMm = 600;
  uint16_t tlmHz = 50;
  bool rcEnable = false;

  // Bateria (divisor 10k / 3.3k -> fator 4.03)
  float batteryDivider = 4.0303f;
  float batteryMinV = 6.8f;
  float batteryCriticalV = 6.4f;
};

// ---------------------------------------------------------------------------
// Temporização
// ---------------------------------------------------------------------------

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t PWM_FREQ_HZ = 50;
constexpr uint32_t ECHO_TIMEOUT_US = 25000;   ///< ~4 m ida e volta
constexpr uint32_t SENSOR_SETTLE_US = 5000;   ///< intervalo entre disparos
constexpr uint32_t BATTERY_PERIOD_MS = 200;
constexpr uint32_t LED_PERIOD_MS = 50;

// Conversão do tempo de eco em milímetros.
// Som a ~343 m/s: mm = us * 343 / 2 / 1000 = us / 5.831
constexpr float US_TO_MM = 1.0f / 5.831f;
constexpr uint16_t DISTANCE_MAX_MM = 4000;

// ---------------------------------------------------------------------------
// Padrões de piscada do LED de status
// ---------------------------------------------------------------------------
// Sem SSH na prova, o LED é a única saída de diagnóstico do carro.
//
//   apagado        .... sem alimentação ou travado
//   1 piscada/s    .... aguardando $CFG (CFG_PENDING)
//   2 piscadas/s   .... pronto, desarmado
//   aceso fixo     .... armado, comando ativo
//   piscada rápida .... FAILSAFE (sem comando do Jetson)
//   3 piscadas/s   .... bateria crítica

enum class LedPattern : uint8_t {
  WaitingConfig,
  Ready,
  Armed,
  Failsafe,
  BatteryCritical,
};
