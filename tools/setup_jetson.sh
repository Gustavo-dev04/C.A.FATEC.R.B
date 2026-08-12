#!/usr/bin/env bash
#
# Preparação do Jetson Orin Nano para o C.A.FATEC.R.B.
#
#   bash tools/setup_jetson.sh
#
# Idempotente: pode rodar de novo sem estragar nada.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

info()  { printf '\033[1;34m==>\033[0m %s\n' "$*"; }
warn()  { printf '\033[1;33m[aviso]\033[0m %s\n' "$*"; }
error() { printf '\033[1;31m[erro]\033[0m %s\n' "$*" >&2; }

# --- 1. Confere que estamos mesmo num Jetson -------------------------------
if [[ -r /proc/device-tree/model ]]; then
    BOARD="$(tr -d '\0' < /proc/device-tree/model)"
    info "placa detectada: $BOARD"
else
    warn "não parece um Jetson. O script segue, mas a câmera CSI não vai funcionar."
fi

# --- 2. Permissão de acesso à porta serial ---------------------------------
# Sem isso, abrir /dev/ttyUSB0 dá "Permission denied" e o erro não é óbvio.
if ! groups "$USER" | grep -qw dialout; then
    info "adicionando $USER ao grupo dialout"
    sudo usermod -aG dialout "$USER"
    warn "faça logout/login (ou reinicie) para o grupo passar a valer"
else
    info "usuário já está no grupo dialout"
fi

# --- 3. Nome fixo para o ESP32 ---------------------------------------------
if [[ ! -f /etc/udev/rules.d/99-robocar.rules ]]; then
    info "instalando regra udev (ESP32 vira /dev/robocar-esp32)"
    sudo cp tools/udev/99-robocar.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger
else
    info "regra udev já instalada"
fi

# --- 4. Dependências Python ------------------------------------------------
info "instalando dependências Python"
python3 -m pip install --user --upgrade pip
python3 -m pip install --user -r jetson/requirements.txt
python3 -m pip install --user -e jetson

# --- 5. OpenCV: o erro clássico do Jetson ----------------------------------
# O JetPack traz OpenCV com CUDA e GStreamer. A versão do pip vem SEM
# GStreamer, e aí a câmera CSI simplesmente não abre — com uma mensagem que
# não explica nada.
info "verificando o OpenCV"
OPENCV_CHECK=$(python3 - <<'PY' 2>/dev/null || echo "ausente"
import re
import sys

try:
    import cv2
except ImportError:
    print("ausente")
    sys.exit(0)

# A linha procurada tem a forma "    GStreamer:                   YES (1.20.3)"
for line in cv2.getBuildInformation().splitlines():
    if re.match(r"\s*GStreamer:", line):
        print("ok" if "YES" in line else "sem-gstreamer")
        break
else:
    print("sem-gstreamer")
PY
)

case "$OPENCV_CHECK" in
    ok)
        info "OpenCV com suporte a GStreamer — ok"
        ;;
    sem-gstreamer)
        error "OpenCV SEM GStreamer: a câmera CSI não vai abrir."
        error "Quase sempre é a versão do pip sobrepondo a do JetPack. Corrija com:"
        error "    python3 -m pip uninstall opencv-python opencv-python-headless"
        ;;
    *)
        error "OpenCV não encontrado. No Jetson ele vem com o JetPack."
        ;;
esac

# --- 6. Desempenho máximo --------------------------------------------------
# Em modo de economia, a câmera perde quadros e o loop de captura não sustenta
# a taxa configurada.
if command -v nvpmodel >/dev/null 2>&1; then
    info "colocando a placa em desempenho máximo"
    sudo nvpmodel -m 0 || warn "nvpmodel falhou (siga assim mesmo)"
    sudo jetson_clocks || warn "jetson_clocks falhou (siga assim mesmo)"
fi

# --- 7. Diretórios de dados ------------------------------------------------
mkdir -p data/sessions data/datasets models
info "diretórios de dados prontos"

# --- 8. Conferência final --------------------------------------------------
echo
info "rodando robocar doctor"
echo
python3 -m robocar.cli doctor || true

echo
info "setup concluído."
echo "Próximos passos:"
echo "  1. Grave o firmware:   cd firmware/esp32 && pio run -t upload"
echo "  2. Teste o enlace:     robocar link --watch"
echo "  3. Teste a câmera:     robocar camera"
echo "  4. Leia:               docs/04-coleta-de-dados.md"
