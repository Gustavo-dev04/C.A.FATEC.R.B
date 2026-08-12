/**
 * Stub mínimo do Arduino.h para compilar e testar o codec do protocolo na
 * máquina de desenvolvimento, sem ESP32.
 *
 * Só existe o que `protocol.cpp` realmente usa. Se um teste de host precisar
 * de mais alguma coisa do Arduino, é sinal de que a lógica testada está
 * acoplada demais ao hardware e provavelmente deveria ser separada.
 */
#pragma once

#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

/// Captura o que o firmware "envia" pela serial, para inspeção nos testes.
class FakeSerial {
 public:
  std::string output;

  void write(const char* data, size_t length) { output.append(data, length); }
  void print(const char* data) { output.append(data); }
  int available() { return static_cast<int>(input.size() - readPos); }
  int read() {
    return readPos < input.size() ? static_cast<uint8_t>(input[readPos++]) : -1;
  }

  void feed(const std::string& data) {
    input += data;
  }
  void clear() {
    output.clear();
    input.clear();
    readPos = 0;
  }

  std::string input;
  size_t readPos = 0;
};

using Stream = FakeSerial;

extern FakeSerial Serial;
