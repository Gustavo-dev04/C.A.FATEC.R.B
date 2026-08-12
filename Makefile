# Atalhos de desenvolvimento do C.A.FATEC.R.B.
#
#   make check    lint + todos os testes (rode antes de commitar)
#   make test     testes Python + testes de host do firmware
#   make firmware compila o firmware do ESP32 (precisa do PlatformIO)

PYTHON ?= python3
JETSON := jetson
ML     := ml
FW     := firmware/esp32

.PHONY: help check test test-python test-firmware lint format \
        firmware upload monitor install install-dev clean

help:
	@echo "C.A.FATEC.R.B — alvos disponíveis:"
	@echo ""
	@echo "  make install       instala o pacote do Jetson (editável)"
	@echo "  make install-dev   instala com as ferramentas de desenvolvimento"
	@echo "  make check         lint + todos os testes"
	@echo "  make test          testes Python + testes de host do firmware"
	@echo "  make lint          ruff"
	@echo "  make format        ruff --fix"
	@echo "  make firmware      compila o firmware do ESP32"
	@echo "  make upload        grava o firmware no ESP32"
	@echo "  make monitor       monitor serial do ESP32"
	@echo "  make clean         remove artefatos de build"

install:
	$(PYTHON) -m pip install -e $(JETSON)

install-dev:
	$(PYTHON) -m pip install -e "$(JETSON)[dev]"
	$(PYTHON) -m pip install -r $(JETSON)/requirements-dev.txt

check: lint test

test: test-python test-firmware

test-python:
	@echo "==> testes Python"
	cd $(JETSON) && $(PYTHON) -m pytest -q

# Compila o codec do protocolo no PC e confere que os bytes batem com os do
# Python. É o que impede que os dois lados saiam de sincronia sem ninguém ver.
test-firmware:
	@echo "==> testes de host do firmware"
	$(MAKE) -C $(FW)/test/host test

lint:
	@echo "==> ruff"
	cd $(JETSON) && ruff check .
	cd $(ML) && ruff check . || true

format:
	cd $(JETSON) && ruff check --fix . && ruff format .

firmware:
	cd $(FW) && pio run

upload:
	cd $(FW) && pio run -t upload

monitor:
	cd $(FW) && pio device monitor

clean:
	$(MAKE) -C $(FW)/test/host clean
	find . -type d -name __pycache__ -prune -exec rm -rf {} +
	find . -type d -name .pytest_cache -prune -exec rm -rf {} +
	find . -type d -name .ruff_cache -prune -exec rm -rf {} +
	rm -rf $(FW)/.pio
