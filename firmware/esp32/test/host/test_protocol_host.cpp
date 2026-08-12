/**
 * Testes de host do codec do protocolo (lado C++).
 *
 * Estes testes existem por um motivo específico: garantir que a implementação
 * em C++ e a em Python produzam e aceitem EXATAMENTE os mesmos bytes. Uma
 * divergência entre as duas não aparece em compilação nem em teste unitário
 * de um lado só — ela aparece como "o carro não responde" na véspera da
 * competição.
 *
 * Os vetores literais aqui são os mesmos de docs/03-protocolo-serial.md e de
 * jetson/tests/test_protocol.py.
 *
 * Rodar:  make -C firmware/esp32/test/host
 */

#include <cassert>
#include <cstdio>
#include <string>

#include "../../include/protocol.h"

FakeSerial Serial;

static int failures = 0;
static int checks = 0;

#define CHECK(cond, msg)                                             \
  do {                                                               \
    ++checks;                                                        \
    if (!(cond)) {                                                   \
      std::printf("  FALHA: %s (linha %d)\n", msg, __LINE__);        \
      ++failures;                                                    \
    }                                                                \
  } while (0)

#define CHECK_STR(actual, expected, msg)                                     \
  do {                                                                       \
    ++checks;                                                                \
    if (std::string(actual) != std::string(expected)) {                      \
      std::printf("  FALHA: %s\n    esperado: '%s'\n    recebido: '%s'\n",   \
                  msg, std::string(expected).c_str(),                        \
                  std::string(actual).c_str());                              \
      ++failures;                                                            \
    }                                                                        \
  } while (0)

static void test_checksum() {
  std::printf("checksum\n");
  // Mesmo vetor de jetson/tests/test_protocol.py::test_checksum_conhecido
  const char* payload = "CMD,142,-0.3125,0.2000,2";
  CHECK(frameChecksum(payload, strlen(payload)) == 0x65,
        "checksum de $CMD precisa ser 0x65 (igual ao Python)");
  CHECK(frameChecksum("", 0) == 0x00, "payload vazio -> 0x00");

  const char* tlm = "TLM,142,84213,-0.3000,0.2000,1,1230,890,1540,7.92,01";
  CHECK(frameChecksum(tlm, strlen(tlm)) == 0x61,
        "checksum de $TLM precisa ser 0x61 (igual ao Python)");
}

static void test_send_frame() {
  std::printf("sendFrame\n");
  Serial.clear();
  sendFrame("CMD,142,-0.3125,0.2000,2");
  CHECK_STR(Serial.output, "$CMD,142,-0.3125,0.2000,2*65\n",
            "quadro codificado precisa ser byte a byte igual ao do Python");
}

static void test_send_telemetry() {
  std::printf("sendTelemetry\n");
  Serial.clear();
  const uint16_t distances[3] = {1230, 890, 1540};
  sendTelemetry(142, 84213, -0.3f, 0.2f, Source::Serial, distances, 3, 7.92f,
                Flag::Armed);
  CHECK_STR(Serial.output,
            "$TLM,142,84213,-0.3000,0.2000,1,1230,890,1540,7.92,01*61\n",
            "telemetria precisa bater com o exemplo de docs/03");
}

static void test_parse_valid() {
  std::printf("parseFrame - quadro válido\n");
  char line[] = "$CMD,142,-0.3125,0.2000,2*65";
  Frame frame;
  uint8_t err = 0;
  CHECK(parseFrame(line, frame, err), "quadro válido deve ser aceito");
  CHECK_STR(frame.type, "CMD", "tipo");
  CHECK(frame.fieldCount == 4, "CMD tem 4 campos");
  CHECK_STR(frame.fields[0], "142", "seq");
  CHECK_STR(frame.fields[1], "-0.3125", "steer");
  CHECK_STR(frame.fields[3], "2", "mode");
}

static void test_parse_bad_checksum() {
  std::printf("parseFrame - checksum inválido\n");
  char line[] = "$CMD,142,-0.3125,0.2000,2*FF";
  Frame frame;
  uint8_t err = 0;
  CHECK(!parseFrame(line, frame, err), "checksum errado deve ser rejeitado");
  CHECK(err == ErrCode::BadChecksum, "código de erro deve ser BadChecksum");
}

static void test_parse_garbage_prefix() {
  std::printf("parseFrame - lixo de boot antes do '$'\n");
  // O bootloader do ESP32 imprime nesta mesma UART antes do firmware assumir.
  char line[] = "rst:0x1 (POWERON_RESET)$PING,7*0B";
  Frame frame;
  uint8_t err = 0;
  CHECK(parseFrame(line, frame, err), "lixo antes do '$' deve ser ignorado");
  CHECK_STR(frame.type, "PING", "tipo após o lixo");
}

static void test_parse_no_star() {
  std::printf("parseFrame - sem '*'\n");
  char line[] = "$CMD,1,0,0,2";
  Frame frame;
  uint8_t err = 0;
  CHECK(!parseFrame(line, frame, err), "quadro sem '*' deve ser rejeitado");
  CHECK(err == ErrCode::BadChecksum, "erro esperado");
}

static void test_line_reader_full() {
  std::printf("LineReader - montagem por pedaços\n");
  LineReader localReader;
  Serial.clear();

  // A serial entrega pedaços arbitrários; o leitor precisa remontar.
  Serial.feed("$PING,");
  CHECK(localReader.poll(Serial) == nullptr, "sem '\\n' ainda não há linha");

  Serial.feed("7*0B\n");
  char* line = localReader.poll(Serial);
  CHECK(line != nullptr, "linha completa deve ser devolvida");
  if (line != nullptr) {
    CHECK_STR(line, "$PING,7*0B", "conteúdo da linha montada");
  }
}

static void test_line_reader_overflow() {
  std::printf("LineReader - linha grande demais\n");
  LineReader localReader;
  Serial.clear();
  std::string huge(MAX_FRAME_LEN + 50, 'x');
  Serial.feed(huge + "\n");
  CHECK(localReader.poll(Serial) == nullptr,
        "linha maior que o buffer deve ser descartada inteira");
  CHECK(localReader.overflows() == 1, "overflow deve ser contabilizado");

  // E o leitor precisa continuar funcionando depois disso.
  Serial.feed("$PING,7*0B\n");
  CHECK(localReader.poll(Serial) != nullptr,
        "leitor deve se recuperar após overflow");
}

static void test_clampf() {
  std::printf("clampf\n");
  CHECK(clampf(5.0f, -1.0f, 1.0f) == 1.0f, "limite superior");
  CHECK(clampf(-5.0f, -1.0f, 1.0f) == -1.0f, "limite inferior");
  CHECK(clampf(0.5f, -1.0f, 1.0f) == 0.5f, "dentro da faixa");
}

int main() {
  std::printf("=== testes de host do protocolo (C++) ===\n\n");

  test_checksum();
  test_send_frame();
  test_send_telemetry();
  test_parse_valid();
  test_parse_bad_checksum();
  test_parse_garbage_prefix();
  test_parse_no_star();
  test_line_reader_full();
  test_line_reader_overflow();
  test_clampf();

  std::printf("\n%d verificações, %d falha(s)\n", checks, failures);
  if (failures == 0) {
    std::printf("OK — o C++ e o Python falam o mesmo protocolo.\n");
  }
  return failures == 0 ? 0 : 1;
}
