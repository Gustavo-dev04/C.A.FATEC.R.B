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

import cv2
import numpy as np

from treino.avaliacao import rodar
from treino.rotulos import MANOBRAS


class PreditorOnnx:
    def __init__(self, caminho: Path) -> None:
        import onnxruntime as ort

        meta = json.loads(Path(caminho).with_suffix(".json").read_text(encoding="utf-8"))
        self.h, self.w = meta["altura"], meta["largura"]
        prov = [p for p in ("CUDAExecutionProvider", "CPUExecutionProvider") if p in ort.get_available_providers()]
        self.sess = ort.InferenceSession(str(caminho), providers=prov)

    def __call__(self, img_bgr: np.ndarray, manobra: str) -> np.ndarray:
        img = cv2.cvtColor(cv2.resize(img_bgr, (self.w, self.h), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)
        x = (img.astype(np.float32) / 255.0).transpose(2, 0, 1)[None]
        m = np.zeros((1, 3), np.float32)
        m[0, MANOBRAS.index(manobra)] = 1.0
        return self.sess.run(None, {"imagem": x, "manobra": m})[0][0]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--onnx", type=Path)
    ap.add_argument("--fonte", choices=["rede", "verdade"], default="rede")
    ap.add_argument("--sementes", default="100000:100010", help="ini:fim (fim exclusivo)")
    ap.add_argument("--saida", type=Path, default=None, help="grava o resultado em JSON")
    a = ap.parse_args(argv)
    ini, fim = (int(v) for v in a.sementes.split(":"))
    if a.fonte == "rede" and a.onnx is None:
        ap.error("--onnx é obrigatório com --fonte rede")
    preditor = PreditorOnnx(a.onnx) if a.fonte == "rede" else None
    r = rodar(preditor, list(range(ini, fim)))
    resumo = {k: v for k, v in r.items() if k != "por_rota"}
    print(json.dumps(resumo, indent=2, ensure_ascii=False))
    if a.saida:
        a.saida.write_text(json.dumps(r, indent=2, ensure_ascii=False), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
