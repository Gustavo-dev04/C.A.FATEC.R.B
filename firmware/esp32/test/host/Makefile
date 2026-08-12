# Testes de host do codec do protocolo.
#
# Compila protocol.cpp na máquina de desenvolvimento (sem ESP32) contra um
# stub de Arduino.h e confere que os bytes gerados batem com os do Python.
#
#   make        compila e roda
#   make clean

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Wno-unused-parameter -O1 -I. -I../../include
TARGET   := test_protocol_host
SOURCES  := test_protocol_host.cpp ../../src/protocol.cpp

.PHONY: all test clean

all: test

$(TARGET): $(SOURCES) Arduino.h ../../include/protocol.h
	$(CXX) $(CXXFLAGS) -o $@ $(SOURCES)

test: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)
