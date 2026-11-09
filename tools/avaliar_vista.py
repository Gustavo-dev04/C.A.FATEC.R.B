"""Avalia o Vista COMPLETO dirigindo no simulador — só com os sensores do carro.

    # o teto: rede de faixa e detector perfeitos (testa mapa, odometria e regras)
    python -m tools.avaliar_vista --sementes 100000:100020 --workers 8
    # a rede de faixa real, detector perfeito (isola a rede)
    python -m tools.avaliar_vista --faixa modelos/faixa.onnx --sementes 100000:100020
    # tudo real: rede de faixa + YOLO
    python -m tools.avaliar_vista --faixa modelos/faixa.onnx --detector modelos/placas.onnx

Na faixa, a rede dirige; no cruzamento, a curva do mapa é seguida por
odometria; a manobra sai da placa DETECTADA + ``rules.decide``; a largada, do
semáforo detectado. Ver ``vista/`` e o ADR 008.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from treino.avaliacao import rodar_vista


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--faixa", default=None, help=".onnx ou .pt da rede de faixa (padrão: verdade)")
    ap.add_argument("--detector", default="verdade", help=".onnx do YOLO (padrão: verdade)")
    ap.add_argument("--sementes", default="100000:100010", help="ini:fim (fim exclusivo)")
    ap.add_argument("--velocidade", type=float, default=0.5, help="velocidade de cruzeiro (m/s)")
    ap.add_argument("--workers", type=int, default=1)
    ap.add_argument("--pitch-graus", type=float, default=None, help="experimento: outra montagem da câmera")
    ap.add_argument("--saida", type=Path, default=None, help="grava o resultado em JSON")
    a = ap.parse_args(argv)
    ini, fim = (int(v) for v in a.sementes.split(":"))
    r = rodar_vista(list(range(ini, fim)), a.faixa, a.detector, v_cruzeiro=a.velocidade, workers=a.workers,
                    pitch_graus=a.pitch_graus)
    print(json.dumps({k: v for k, v in r.items() if k != "por_rota"}, indent=2, ensure_ascii=False))
    if a.saida:
        a.saida.write_text(json.dumps(r, indent=2, ensure_ascii=False), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
