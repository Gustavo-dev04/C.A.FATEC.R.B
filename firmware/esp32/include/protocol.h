/**
 * Codec do protocolo serial — lado ESP32.
 *
 * Espelha jetson/robocar/comms/protocol.py. Especificação normativa em
 * docs/03-protocolo-serial.md. Mudou de um lado, muda dos dois e atualiza
 * jetson/tests/test_protocol.py na mesma alteração.
 */
#pragma once

#include <Arduino.h>

constexpr uint8_t PROTOCOL_VERSION = 1;
constexpr size_t MAX_FRAME_LEN = 128;
constexpr uint8_t MAX_FIELDS = 16;

// Modo declarado pelo Jetson em $CMD
enum class Mode : uint8_t { Idle = 0, Teleop = 1, Auto = 2 };

// Origem do comando efetivamente aplicado, reportada em $TLM
enum class Source : uint8_t { Failsafe = 0, Serial = 1, Rc = 2 };

// Bitfield de estado (campo `flags` de $TLM)
namespace Flag {
constexpr uint8_t Armed = 0x01;
constexpr uint8_t Failsafe = 0x02;
constexpr uint8_t RcValid = 0x04;
constexpr uint8_t ObstacleStop = 0x08;
constexpr uint8_t ObstacleSlow = 0x10;
constexpr uint8_t BattLow = 0x20;
constexpr uint8_t BattCritical = 0x40;
constexpr uint8_t CfgPending = 0x80;
}  // namespace Flag

// Códigos de $ERR
namespace ErrCode {
constexpr uint8_t BadChecksum = 1;
constexpr uint8_t UnknownType = 2;
constexpr uint8_t BadFieldCount = 3;
constexpr uint8_t OutOfRange = 4;
constexpr uint8_t UnknownConfigKey = 5;
constexpr uint8_t FrameTooLong = 6;
}  // namespace ErrCode

/// Um quadro já validado e dividido em campos.
struct Frame {
  const char* type = nullptr;
  const char* fields[MAX_FIELDS] = {nullptr};
  uint8_t fieldCount = 0;
};

/// XOR de todos os bytes do payload (o conteúdo entre '$' e '*').
uint8_t frameChecksum(const char* payload, size_t len);

/**
 * Valida e divide um quadro recebido.
 *
 * @param line  buffer terminado em '\0'. É MODIFICADO no lugar: as vírgulas
 *              viram '\0' para delimitar os campos, evitando cópia.
 * @param out   preenchido em caso de sucesso.
 * @param err   código de erro quando devolve false.
 * @return true se o quadro é válido.
 */
bool parseFrame(char* line, Frame& out, uint8_t& err);

/// Envia um quadro montando '$', checksum e '\n'.
void sendFrame(const char* payload);

/// Formata e envia $TLM. `distances` tem `count` elementos, em mm.
void sendTelemetry(uint16_t seq, uint32_t tMs, float steer, float throttle,
                   Source source, const uint16_t* distances, uint8_t count,
                   float vbat, uint8_t flags);

void sendPong(uint16_t seq, uint32_t tMs);
void sendAck(const char* kind, const char* detail);
void sendErr(uint8_t code, const char* detail);
void sendLog(char level, const char* text);

/// Limita `v` ao intervalo [lo, hi].
inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

/**
 * Acumula bytes da serial e devolve linhas completas.
 *
 * Descarta silenciosamente o que vier antes do '$' e o que passar de
 * MAX_FRAME_LEN, para que ruído na linha nunca trave o loop de controle.
 */
class LineReader {
 public:
  /// @return ponteiro para a linha completa (sem '\n'), ou nullptr.
  char* poll(Stream& stream);
  void reset() { length_ = 0; overflow_ = false; }
  uint32_t overflows() const { return overflowCount_; }

 private:
  char buffer_[MAX_FRAME_LEN + 1] = {0};
  size_t length_ = 0;
  bool overflow_ = false;
  uint32_t overflowCount_ = 0;
};
