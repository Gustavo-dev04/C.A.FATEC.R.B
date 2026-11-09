"""Avalia a rede de faixa (ONNX) dirigindo no simulador.

    python -m tools.avaliar_modelo --onnx modelos/faixa.onnx --sementes 100000:100020
    python -m tools.avaliar_modelo --fonte verdade --sementes 100000:100020   # o teto

Usa o MESMO arquivo ONNX que vai para a Jetson (antes do TensorRT), via
onnxruntime. Semáforo e manobra vêm da verdade nesta etapa (``treino.avaliacao``).
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from treino.avaliacao import PreditorOnnx, rodar


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--onnx", type=Path)
    ap.add_argument("--fonte", choices=["rede", "verdade"], default="rede")
    ap.add_argument("--sementes", default="100000:100010", help="ini:fim (fim exclusivo)")
    ap.add_argument("--sem-filtro", action="store_true", help="rede direto no controle, sem o filtro")
    ap.add_argument("--saida", type=Path, default=None, help="grava o resultado em JSON")
    a = ap.parse_args(argv)
    ini, fim = (int(v) for v in a.sementes.split(":"))
    if a.fonte == "rede" and a.onnx is None:
        ap.error("--onnx é obrigatório com --fonte rede")
    preditor = PreditorOnnx(a.onnx) if a.fonte == "rede" else None
    r = rodar(preditor, list(range(ini, fim)), filtro=not a.sem_filtro)
    resumo = {k: v for k, v in r.items() if k != "por_rota"}
    print(json.dumps(resumo, indent=2, ensure_ascii=False))
    if a.saida:
        a.saida.write_text(json.dumps(r, indent=2, ensure_ascii=False), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
