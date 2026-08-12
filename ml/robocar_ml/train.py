"""Treinamento do modelo de direção.

    python -m robocar_ml.train --data ../data/sessions --epochs 40

Roda tanto no Jetson quanto num PC com GPU. Treinar no Jetson funciona, mas é
lento — o normal é treinar no PC e levar só o ``.onnx`` para o carro.
"""

from __future__ import annotations

import argparse
import json
import logging
import time
from pathlib import Path

import torch
import torch.nn as nn
from torch.utils.data import DataLoader

from .dataset import (
    DrivingDataset,
    balance_samples,
    load_samples,
    split_by_session,
    steering_histogram,
)
from .models import build_model, count_parameters

log = logging.getLogger("train")


def evaluate(model: nn.Module, loader: DataLoader, criterion, device: str) -> dict:
    """Perda e erro absoluto médio no conjunto de validação.

    O MAE em unidades de esterço é o número que interessa: 0.05 significa que
    o modelo erra, em média, 5% do curso total do servo. Acima de ~0.15 o carro
    não segue faixa de forma confiável.
    """
    model.eval()
    total_loss = 0.0
    total_abs = 0.0
    count = 0

    with torch.no_grad():
        for images, targets in loader:
            images = images.to(device, non_blocking=True)
            targets = targets.to(device, non_blocking=True)
            outputs = model(images)
            total_loss += criterion(outputs, targets).item() * images.size(0)
            total_abs += (outputs - targets).abs().sum().item()
            count += images.size(0)

    divisor = max(count, 1)
    return {
        "loss": total_loss / divisor,
        "mae": total_abs / (divisor * loader.dataset[0][1].numel()),
    }


def train(args: argparse.Namespace) -> int:
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S",
    )
    torch.manual_seed(args.seed)

    device = args.device or ("cuda" if torch.cuda.is_available() else "cpu")
    log.info("dispositivo: %s", device)
    if device == "cpu":
        log.warning("treinando na CPU — vai demorar. Use uma máquina com GPU.")

    # --- dados -------------------------------------------------------------
    samples = load_samples(
        [Path(p) for p in args.data], min_throttle=args.min_throttle
    )
    log.info("%d amostras brutas", len(samples))

    counts, edges = steering_histogram(samples)
    center_bin = len(counts) // 2
    center_fraction = counts[center_bin] / max(counts.sum(), 1)
    log.info("fração de amostras na faixa central de esterço: %.1f%%", center_fraction * 100)
    if center_fraction > 0.5 and not args.balance:
        log.warning(
            "mais da metade das amostras está perto de esterço zero e o "
            "balanceamento está DESLIGADO. O modelo tende a aprender 'vai "
            "reto'. Considere --balance."
        )

    if args.balance:
        samples = balance_samples(samples, keep_factor=args.keep_factor, seed=args.seed)

    train_samples, val_samples = split_by_session(
        samples, val_fraction=args.val_fraction, seed=args.seed
    )

    outputs = 2 if args.predict_throttle else 1
    common = {
        "width": args.width,
        "height": args.height,
        "predict_throttle": args.predict_throttle,
        "horizontal_flip": args.horizontal_flip,
    }
    train_set = DrivingDataset(train_samples, augment=True, **common)
    val_set = DrivingDataset(val_samples, augment=False, **common)

    train_loader = DataLoader(
        train_set,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.workers,
        pin_memory=(device == "cuda"),
        drop_last=True,
    )
    val_loader = DataLoader(
        val_set,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=(device == "cuda"),
    )

    # --- modelo ------------------------------------------------------------
    model = build_model(args.model, outputs=outputs, dropout=args.dropout).to(device)
    log.info("%s: %d parâmetros treináveis", args.model, count_parameters(model))

    # Huber (SmoothL1) em vez de MSE: um rótulo ruim isolado — o piloto deu um
    # esterço brusco para corrigir algo — não domina o gradiente.
    criterion = nn.SmoothL1Loss(beta=0.1)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode="min", factor=0.5, patience=4
    )

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    best_loss = float("inf")
    epochs_without_improvement = 0
    history = []

    for epoch in range(1, args.epochs + 1):
        model.train()
        started = time.time()
        running = 0.0
        seen = 0

        for images, targets in train_loader:
            images = images.to(device, non_blocking=True)
            targets = targets.to(device, non_blocking=True)

            optimizer.zero_grad(set_to_none=True)
            loss = criterion(model(images), targets)
            loss.backward()
            # O clip evita que um lote com rótulo ruim exploda os pesos.
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()

            running += loss.item() * images.size(0)
            seen += images.size(0)

        train_loss = running / max(seen, 1)
        metrics = evaluate(model, val_loader, criterion, device)
        scheduler.step(metrics["loss"])

        log.info(
            "época %3d/%d  treino=%.5f  val=%.5f  mae=%.4f  lr=%.2e  %.0fs",
            epoch,
            args.epochs,
            train_loss,
            metrics["loss"],
            metrics["mae"],
            optimizer.param_groups[0]["lr"],
            time.time() - started,
        )
        history.append(
            {
                "epoch": epoch,
                "train_loss": train_loss,
                "val_loss": metrics["loss"],
                "val_mae": metrics["mae"],
            }
        )

        if metrics["loss"] < best_loss:
            best_loss = metrics["loss"]
            epochs_without_improvement = 0
            checkpoint = {
                "model_state": model.state_dict(),
                "model_name": args.model,
                "outputs": outputs,
                "input_size": [args.height, args.width],
                "val_loss": best_loss,
                "val_mae": metrics["mae"],
                "epoch": epoch,
                "sessions_train": sorted({s.session for s in train_samples}),
                "sessions_val": sorted({s.session for s in val_samples}),
            }
            torch.save(checkpoint, output_dir / "best.pt")
            log.info("  -> novo melhor modelo salvo (val=%.5f)", best_loss)
        else:
            epochs_without_improvement += 1
            if epochs_without_improvement >= args.patience:
                log.info("parada antecipada: %d épocas sem melhora", args.patience)
                break

    (output_dir / "history.json").write_text(json.dumps(history, indent=2))
    log.info("melhor perda de validação: %.5f", best_loss)
    log.info("modelo em %s", output_dir / "best.pt")
    log.info(
        "próximo passo: exportar para ONNX e depois TensorRT — "
        "python -m robocar_ml.export --checkpoint %s", output_dir / "best.pt"
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Treina o modelo de direção do C.A.FATEC.R.B")
    parser.add_argument("--data", nargs="+", default=["../data/sessions"],
                        help="diretórios com as sessões coletadas")
    parser.add_argument("--output", default="../models/steering")
    parser.add_argument("--model", default="pilotnet")

    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--dropout", type=float, default=0.3)
    parser.add_argument("--patience", type=int, default=10)

    parser.add_argument("--width", type=int, default=200)
    parser.add_argument("--height", type=int, default=66)
    parser.add_argument("--val-fraction", type=float, default=0.2)
    parser.add_argument("--min-throttle", type=float, default=0.03)

    parser.add_argument("--balance", action="store_true",
                        help="reduz a super-representação de esterço ~0 (recomendado)")
    parser.add_argument("--keep-factor", type=float, default=1.5)
    parser.add_argument("--horizontal-flip", action="store_true",
                        help="ATENÇÃO: inverte a semântica de 'manter-se à direita'")
    parser.add_argument("--predict-throttle", action="store_true")

    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--device", default=None)
    parser.add_argument("--seed", type=int, default=42)
    return parser


def main(argv: list[str] | None = None) -> int:
    return train(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
