#include "protocol.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

namespace {

/// Comparação de 2 dígitos hex sem depender de strncasecmp, cuja declaração
/// varia entre toolchains (glibc, newlib do ESP32, etc.).
bool hexPairEquals(const char* a, const char* b) {
  for (int i = 0; i < 2; ++i) {
    if (toupper(static_cast<unsigned char>(a[i])) !=
        toupper(static_cast<unsigned char>(b[i]))) {
      return false;
    }
  }
  return true;
}

}  // namespace

uint8_t frameChecksum(const char* payload, size_t len) {
  uint8_t cs = 0;
  for (size_t i = 0; i < len; ++i) {
    cs ^= static_cast<uint8_t>(payload[i]);
  }
  return cs;
}

bool parseFrame(char* line, Frame& out, uint8_t& err) {
  // Lixo antes do '$' é normal: o bootloader do ESP32 imprime na mesma UART.
  char* start = strchr(line, '$');
  if (start == nullptr) {
    err = ErrCode::BadFieldCount;
    return false;
  }
  char* payload = start + 1;

  char* star = strrchr(payload, '*');
  if (star == nullptr) {
    err = ErrCode::BadChecksum;
    return false;
  }
  *star = '\0';
  const char* received = star + 1;

  if (strlen(received) != 2) {
    err = ErrCode::BadChecksum;
    return false;
  }

  char expected[3];
  snprintf(expected, sizeof(expected), "%02X",
           frameChecksum(payload, strlen(payload)));
  if (!hexPairEquals(expected, received)) {
    err = ErrCode::BadChecksum;
    return false;
  }

  // Divide no lugar: as vírgulas viram '\0'. Sem alocação, sem cópia.
  out.fieldCount = 0;
  out.type = payload;
  for (char* cursor = payload; *cursor != '\0'; ++cursor) {
    if (*cursor == ',') {
      *cursor = '\0';
      if (out.fieldCount >= MAX_FIELDS) {
        err = ErrCode::BadFieldCount;
        return false;
      }
      out.fields[out.fieldCount++] = cursor + 1;
    }
  }
  return true;
}

void sendFrame(const char* payload) {
  char frame[MAX_FRAME_LEN];
  int written = snprintf(frame, sizeof(frame), "$%s*%02X\n", payload,
                         frameChecksum(payload, strlen(payload)));
  if (written > 0 && static_cast<size_t>(written) < sizeof(frame)) {
    Serial.write(frame, written);
  }
}

void sendTelemetry(uint16_t seq, uint32_t tMs, float steer, float throttle,
                   Source source, const uint16_t* distances, uint8_t count,
                   float vbat, uint8_t flags) {
  char payload[MAX_FRAME_LEN - 8];
  int offset = snprintf(payload, sizeof(payload), "TLM,%u,%lu,%.4f,%.4f,%u", seq,
                        static_cast<unsigned long>(tMs), steer, throttle,
                        static_cast<unsigned>(source));
  for (uint8_t i = 0; i < count && offset > 0; ++i) {
    offset += snprintf(payload + offset, sizeof(payload) - offset, ",%u",
                       distances[i]);
  }
  if (offset > 0 && static_cast<size_t>(offset) < sizeof(payload)) {
    snprintf(payload + offset, sizeof(payload) - offset, ",%.2f,%02X", vbat,
             flags);
    sendFrame(payload);
  }
}

void sendPong(uint16_t seq, uint32_t tMs) {
  char payload[32];
  snprintf(payload, sizeof(payload), "PONG,%u,%lu", seq,
           static_cast<unsigned long>(tMs));
  sendFrame(payload);
}

void sendAck(const char* kind, const char* detail) {
  char payload[MAX_FRAME_LEN - 8];
  snprintf(payload, sizeof(payload), "ACK,%s,%s", kind, detail ? detail : "");
  sendFrame(payload);
}

void sendErr(uint8_t code, const char* detail) {
  char payload[MAX_FRAME_LEN - 8];
  snprintf(payload, sizeof(payload), "ERR,%u,%s", code, detail ? detail : "");
  sendFrame(payload);
}

void sendLog(char level, const char* text) {
  char payload[MAX_FRAME_LEN - 8];
  snprintf(payload, sizeof(payload), "LOG,%c,%s", level, text);
  // Vírgula e '*' quebrariam o quadro do outro lado.
  for (char* c = payload; *c != '\0'; ++c) {
    if (*c == ',' && c > payload + 5) *c = ' ';
    if (*c == '*') *c = ' ';
  }
  sendFrame(payload);
}

char* LineReader::poll(Stream& stream) {
  while (stream.available() > 0) {
    char c = static_cast<char>(stream.read());

    if (c == '\n' || c == '\r') {
      if (length_ == 0) continue;
      if (overflow_) {
        // Linha maior que o buffer: descarta inteira em vez de processar
        // um pedaço, que seria pior do que não processar nada.
        reset();
        continue;
      }
      buffer_[length_] = '\0';
      length_ = 0;
      return buffer_;
    }

    if (length_ >= MAX_FRAME_LEN) {
      if (!overflow_) {
        overflow_ = true;
        ++overflowCount_;
      }
      continue;
    }
    buffer_[length_++] = c;
  }
  return nullptr;
}
