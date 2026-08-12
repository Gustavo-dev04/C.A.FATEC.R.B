"""Exportação do modelo treinado para ONNX (e daí para TensorRT).

    python -m robocar_ml.export --checkpoint ../models/steering/best.pt

No Jetson, converta o ONNX para um engine TensorRT:

    /usr/src/tensorrt/bin/trtexec \\
        --onnx=models/steering/model.onnx \\
        --saveEngine=models/steering/model.engine \\
        --fp16

FP16 costuma dar 2-3x de ganho no Orin Nano com perda desprezível para
regressão de esterço. **O engine é específico da GPU e da versão do TensorRT**
— gere no próprio Jetson que vai correr, nunca no PC de treino.
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

import torch

from .models import build_model

log = logging.getLogger("export")


def export(args: argparse.Namespace) -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)-7s %(message)s")

    checkpoint_path = Path(args.checkpoint)
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)

    model = build_model(
        checkpoint.get("model_name", "pilotnet"),
        outputs=checkpoint.get("outputs", 1),
    )
    model.load_state_dict(checkpoint["model_state"])
    model.eval()

    height, width = checkpoint.get("input_size", [66, 200])
    dummy = torch.randn(1, 3, height, width)

    output_path = Path(args.output or checkpoint_path.with_suffix(".onnx"))
    output_path.parent.mkdir(parents=True, exist_ok=True)

    torch.onnx.export(
        model,
        dummy,
        str(output_path),
        input_names=["image"],
        output_names=["control"],
        # Lote dinâmico permite avaliar em lote no PC e rodar com lote 1 no
        # carro usando o mesmo arquivo.
        dynamic_axes={"image": {0: "batch"}, "control": {0: "batch"}},
        opset_version=args.opset,
    )

    log.info("exportado: %s", output_path)
    log.info("entrada: 1x3x%dx%d, normalizada em [-1, 1]", height, width)
    log.info("val_loss do checkpoint: %.5f", checkpoint.get("val_loss", float("nan")))
    log.info("val_mae do checkpoint:  %.4f", checkpoint.get("val_mae", float("nan")))

    if args.verify:
        _verify(output_path, model, dummy)

    print("\nNo Jetson, gere o engine TensorRT:")
    print(
        f"  /usr/src/tensorrt/bin/trtexec --onnx={output_path.name} "
        f"--saveEngine={output_path.with_suffix('.engine').name} --fp16"
    )
    return 0


def _verify(onnx_path: Path, model, dummy) -> None:
    """Confere que o ONNX devolve o mesmo que o PyTorch."""
    try:
        import numpy as np
        import onnxruntime
    except ImportError:
        log.warning("onnxruntime não instalado — pulando verificação")
        return

    session = onnxruntime.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    onnx_output = session.run(None, {"image": dummy.numpy()})[0]
    with torch.no_grad():
        torch_output = model(dummy).numpy()

    diff = float(np.abs(onnx_output - torch_output).max())
    log.info("diferença máxima PyTorch vs ONNX: %.2e", diff)
    if diff > 1e-4:
        log.error(
            "divergência acima do tolerável — não use este ONNX antes de investigar"
        )
    else:
        log.info("verificação OK")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Exporta o modelo treinado para ONNX")
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--output", default=None)
    parser.add_argument("--opset", type=int, default=13)
    parser.add_argument("--verify", action="store_true", default=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    return export(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
