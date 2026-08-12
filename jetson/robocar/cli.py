"""Interface de linha de comando do ``robocar``.

    robocar doctor              confere o ambiente antes de sair de casa
    robocar link                testa o enlace com o ESP32
    robocar camera              testa a câmera e mede a taxa real
    robocar record              **grava uma sessão de dataset**
    robocar dataset stats       resumo do que já foi coletado
    robocar dataset verify      confere integridade das sessões
    robocar dataset tag         marca uma sessão (ex.: descartar)
    robocar calib steering      calibra centro e batentes da direção
    robocar calib camera        calibra intrínsecos e distorção da lente

Cada subcomando importa suas dependências pesadas só quando executado, então
``robocar dataset stats`` roda em qualquer máquina, sem OpenCV nem pyserial.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from pathlib import Path

from . import __version__

log = logging.getLogger("robocar")

KB_PER_FRAME = 280
"""Tamanho médio de um quadro JPEG q90 em 1640x1232, usado nas estimativas de
disco. Se mudar a resolução ou a qualidade em ``config/camera.yaml``, meça de
novo com ``du -sh`` numa sessão real e ajuste aqui."""


def _setup_logging(verbose: bool) -> None:
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


# ---------------------------------------------------------------------------
# doctor
# ---------------------------------------------------------------------------


def cmd_doctor(args: argparse.Namespace) -> int:
    """Confere tudo que costuma faltar no dia da competição."""
    from .capture.session import disk_free_gb, system_info
    from .config import ConfigError, find_repo_root, load_config

    ok = True

    def check(label: str, passed: bool, detail: str = "") -> None:
        nonlocal ok
        mark = "  ok  " if passed else " FALHA"
        print(f"[{mark}] {label}" + (f" — {detail}" if detail else ""))
        ok = ok and passed

    print("=== ambiente ===")
    for key, value in system_info().items():
        print(f"  {key}: {value}")

    print("\n=== configuração ===")
    try:
        root = find_repo_root()
        config = load_config(root, profile=args.profile)
        check("config/*.yaml carregado e coerente", True, str(root))
    except ConfigError as exc:
        check("config/*.yaml", False, str(exc))
        return 1

    print("\n=== dependências ===")
    for module, needed_for in (
        ("yaml", "configuração"),
        ("serial", "enlace com o ESP32"),
        ("cv2", "câmera e gravação"),
        ("numpy", "processamento de imagem"),
    ):
        try:
            __import__(module)
            check(f"módulo {module}", True, needed_for)
        except ImportError:
            check(f"módulo {module}", False, f"necessário para {needed_for}")

    print("\n=== disco ===")
    sessions_root = root / config.get_str("capture.root", "data/sessions")
    free_gb = disk_free_gb(sessions_root)
    rate = config.get_int("capture.rate_hz", 20)
    minutes = free_gb * 1024**2 / (rate * 60 * KB_PER_FRAME) if rate else 0
    check(
        f"{free_gb:.1f} GB livres em {sessions_root}",
        free_gb >= 10,
        f"~{minutes:.0f} min de gravação a {rate} Hz",
    )

    print("\n=== serial ===")
    try:
        from .comms.link import find_port

        port = find_port()
        check("porta do ESP32 encontrada", True, port)
    except Exception as exc:
        check("porta do ESP32", False, str(exc))

    print("\n=== câmera ===")
    video_devices = sorted(Path("/dev").glob("video*"))
    check(
        "dispositivo de vídeo presente",
        bool(video_devices),
        ", ".join(d.name for d in video_devices) or "nenhum /dev/video*",
    )

    print("\n" + ("Tudo pronto." if ok else "Há itens pendentes acima."))
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# link
# ---------------------------------------------------------------------------


def cmd_link(args: argparse.Namespace) -> int:
    """Testa o enlace serial e mostra a telemetria em tempo real."""
    from .comms.link import LinkError, SerialLink
    from .comms.protocol import Flags, build_config_frames
    from .config import find_repo_root, load_config, throttle_limit_for

    config = load_config(find_repo_root(), profile=args.profile)

    try:
        with SerialLink(args.port, args.baud) as link:
            rtt = link.ping(timeout=2.0)
            if rtt is None:
                print("ESP32 não respondeu ao $PING.", file=sys.stderr)
                print(
                    "Confira: firmware gravado, cabo de dados (não só de carga), "
                    "baud 115200 e a porta correta.",
                    file=sys.stderr,
                )
                return 1
            print(f"$PONG em {rtt:.2f} ms via {link.port}")

            if args.handshake:
                frames = build_config_frames(
                    config.as_dict(),
                    throttle_limit=throttle_limit_for(config, "collect"),
                    rc_enable=args.rc,
                )
                link.handshake(frames)
                print(f"handshake concluído ({len(frames)} parâmetros enviados)")

            if not args.watch:
                return 0

            print("\nmonitorando telemetria — Ctrl+C para sair\n")
            while True:
                telemetry, age = link.telemetry_with_age()
                if telemetry is None:
                    print("aguardando $TLM...", end="\r")
                else:
                    flags = Flags(telemetry.flags)
                    active = [f.name for f in Flags if f.value and f in flags]
                    distances = " ".join(
                        f"{d:>4}" if d else "   -" for d in telemetry.distances_mm
                    )
                    print(
                        f"steer={telemetry.steer:+.3f} thr={telemetry.throttle:+.3f} "
                        f"src={telemetry.source.name:<8} d=[{distances}] "
                        f"vbat={telemetry.vbat:.2f}V idade={age:5.1f}ms "
                        f"flags={'|'.join(active) or '-'}",
                        end="   \r",
                    )
                # O ESP32 entra em failsafe sem $CMD; mantém o batimento.
                link.send_command(0.0, 0.0)
                time.sleep(0.05)
    except LinkError as exc:
        print(f"erro de enlace: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nencerrado.")
    return 0


# ---------------------------------------------------------------------------
# camera
# ---------------------------------------------------------------------------


def cmd_camera(args: argparse.Namespace) -> int:
    """Abre a câmera, mede a taxa real e opcionalmente ajuda a focar."""
    from .config import find_repo_root, load_config
    from .sensors.camera import CameraError, open_camera
    from .util.rate import LoopMonitor

    config = load_config(find_repo_root(), profile=args.profile)

    try:
        camera = open_camera(config, override_backend=args.backend)
    except CameraError as exc:
        print(f"erro de câmera: {exc}", file=sys.stderr)
        return 1

    print(f"câmera aberta: {camera.width}x{camera.height} @ {camera.fps} fps alvo")
    _warn_if_cropped_mode(camera, config)

    if args.focus:
        return _focus_assistant(camera, config)

    virtual = None
    if args.virtual:
        try:
            virtual = _build_virtual_camera(config, camera)
        except Exception as exc:
            print(f"câmera virtual indisponível: {exc}", file=sys.stderr)
            camera.release()
            return 1
        print(virtual.describe())

    monitor = LoopMonitor()
    print("medindo taxa real — Ctrl+C para sair\n")

    frames = 0
    try:
        with camera:
            while args.frames == 0 or frames < args.frames:
                frame, _ = camera.read()
                monitor.tick()
                frames += 1
                if frames % 10 == 0:
                    print(f"  {monitor.summary()}", end="\r")
                if args.snapshot and frames == 1:
                    import cv2  # type: ignore[import-not-found]

                    path = Path(args.snapshot)
                    path.parent.mkdir(parents=True, exist_ok=True)
                    cv2.imwrite(str(path), frame)
                    print(f"\nquadro salvo em {path}")
                    if virtual is not None:
                        # Salva a vista virtual ao lado, para comparar o
                        # enquadramento e a retificação lado a lado.
                        virtual_path = path.with_name(f"{path.stem}_virtual{path.suffix}")
                        cv2.imwrite(str(virtual_path), virtual.apply(frame))
                        print(f"vista virtual salva em {virtual_path}")
    except KeyboardInterrupt:
        pass
    except CameraError as exc:
        print(f"\nerro de câmera: {exc}", file=sys.stderr)
        return 1

    print(f"\n{frames} quadros — {monitor.summary()}")
    target = config.get_int("camera.capture.fps", 30)
    if monitor.hz < target * 0.85:
        print(
            f"\nATENÇÃO: taxa real ({monitor.hz:.1f} Hz) bem abaixo do alvo "
            f"({target} Hz). Causas comuns: resolução alta demais, CPU em modo "
            f"de economia (rode `sudo nvpmodel -m 0 && sudo jetson_clocks`) ou "
            f"outro processo usando a câmera."
        )
    return 0


#: Modos do IMX219 que são RECORTE do sensor, não redução. Usá-los custa parte
#: do campo de visão da lente de 120° — ver docs/02-hardware.md.
_IMX219_CROPPED_MODES = {(1920, 1080), (1280, 720), (640, 480)}


def _warn_if_cropped_mode(camera, config) -> None:
    if config.get_str("camera.backend", "csi") != "csi":
        return
    if (camera.width, camera.height) in _IMX219_CROPPED_MODES:
        print(
            f"\nATENÇÃO: {camera.width}x{camera.height} é um modo RECORTADO do "
            "IMX219 — você perde parte dos 120° da lente e ganha menos pixels "
            "na placa.\nUse 1640x1232 (campo completo, 30 fps). Ver "
            "docs/02-hardware.md.\n"
        )


def _build_virtual_camera(config, camera):
    """Monta a câmera virtual das placas a partir da calibração salva."""
    from .config import find_repo_root
    from .sensors.lens import CameraIntrinsics, virtual_camera_from_config

    path = find_repo_root() / config.get_str(
        "camera.intrinsics_file", "config/calib/camera_intrinsics.yaml"
    )
    intrinsics = CameraIntrinsics.load(path)

    # A calibração pode ter sido feita em outra resolução do mesmo modo.
    if (intrinsics.width, intrinsics.height) != (camera.width, camera.height):
        print(
            f"  calibração feita em {intrinsics.width}x{intrinsics.height}, "
            f"reescalando para {camera.width}x{camera.height}"
        )
        intrinsics = intrinsics.scale_to(camera.width, camera.height)

    return virtual_camera_from_config(config, intrinsics).build()


def _focus_assistant(camera, config) -> int:
    """Medidor de nitidez ao vivo, para ajustar a rosca de foco da lente."""
    from .sensors.camera import CameraError, sharpness

    reference = config.get_float("camera.focus.reference_sharpness", 0.0)
    distance = config.get_float("camera.focus.target_distance_m", 2.0)
    signs_roi = config.get("camera.roi_signs", None)

    print(
        f"\n=== assistente de foco ===\n"
        f"Aponte a câmera para algo a ~{distance:.1f} m (distância de leitura\n"
        f"das placas) e gire a rosca DEVAGAR. O número sobe conforme foca:\n"
        f"pare quando ele parar de subir e recue até o pico.\n"
    )
    if reference > 0:
        print(f"referência anotada em config/camera.yaml: {reference:.0f}\n")
    print("Ctrl+C encerra e mostra o pico.\n")

    peak = 0.0
    peak_signs = 0.0
    try:
        with camera:
            while True:
                frame, _ = camera.read()
                score = sharpness(frame)
                score_signs = sharpness(frame, signs_roi) if signs_roi else 0.0
                peak = max(peak, score)
                peak_signs = max(peak_signs, score_signs)

                # Barra proporcional ao pico da sessão: dá retorno visual
                # mesmo sem saber de antemão a escala do valor absoluto.
                filled = int(40 * score / peak) if peak > 0 else 0
                bar = "#" * filled + "." * (40 - filled)
                print(
                    f"  nitidez={score:8.0f}  pico={peak:8.0f}  "
                    f"placas={score_signs:8.0f}  [{bar}]",
                    end="\r",
                )
    except KeyboardInterrupt:
        pass
    except CameraError as exc:
        print(f"\nerro de câmera: {exc}", file=sys.stderr)
        return 1

    print(f"\n\npico global:            {peak:.0f}")
    print(f"pico na região das placas: {peak_signs:.0f}")

    if reference > 0:
        ratio = peak / reference
        minimum = config.get_float("camera.focus.min_sharpness_ratio", 0.6)
        print(f"em relação à referência:   {ratio * 100:.0f}%")
        if ratio < minimum:
            print(
                f"\nATENÇÃO: bem abaixo da referência ({reference:.0f}). "
                "A câmera provavelmente desfocou — reajuste a rosca antes de "
                "gravar qualquer sessão."
            )
            return 1
    else:
        print(
            f"\nAnote em config/camera.yaml:\n"
            f"  camera.focus.reference_sharpness: {peak:.0f}\n"
            f"\nDepois TRAVE a rosca (esmalte ou Loctite 243) e marque a "
            f"posição com caneta permanente atravessando lente e corpo."
        )
    return 0


# ---------------------------------------------------------------------------
# record
# ---------------------------------------------------------------------------


def cmd_record(args: argparse.Namespace) -> int:
    """Grava uma sessão de dataset. É o comando principal da coleta."""
    from .capture.recorder import SessionRecorder
    from .capture.session import create_session, disk_free_gb
    from .comms.link import LinkError, SerialLink
    from .comms.protocol import Mode, build_config_frames
    from .config import find_repo_root, load_config, throttle_limit_for
    from .control.sources import make_source
    from .sensors.camera import CameraError, open_camera
    from .util.rate import LoopMonitor, RateLimiter

    root = find_repo_root()
    config = load_config(root, profile=args.profile)

    sessions_root = root / config.get_str("capture.root", "data/sessions")
    sessions_root.mkdir(parents=True, exist_ok=True)

    free_gb = disk_free_gb(sessions_root)
    if free_gb < 5:
        print(
            f"ERRO: apenas {free_gb:.1f} GB livres. Libere espaço antes de gravar.",
            file=sys.stderr,
        )
        return 1
    if free_gb < 20:
        print(f"AVISO: {free_gb:.1f} GB livres — dá para ~{free_gb * 6:.0f} minutos.")

    metadata = _prompt_metadata(config, args)

    # --- abre os subsistemas na ordem em que podem falhar --------------
    try:
        link = SerialLink(args.port, args.baud).open()
    except LinkError as exc:
        print(f"erro de enlace: {exc}", file=sys.stderr)
        return 1

    try:
        frames_cfg = build_config_frames(
            config.as_dict(),
            throttle_limit=throttle_limit_for(config, "collect"),
            rc_enable=(args.control == "rc"),
        )
        link.handshake(frames_cfg)
    except LinkError as exc:
        print(f"erro no handshake: {exc}", file=sys.stderr)
        link.close()
        return 1

    try:
        camera = open_camera(config, override_backend=args.backend)
    except CameraError as exc:
        print(f"erro de câmera: {exc}", file=sys.stderr)
        link.close()
        return 1

    try:
        source = make_source(args.control)
    except (RuntimeError, ValueError) as exc:
        print(f"erro na fonte de controle: {exc}", file=sys.stderr)
        camera.release()
        link.close()
        return 1

    session_dir, meta = create_session(
        sessions_root, config, repo_root=root, tag=args.tag, **metadata
    )

    rate = config.get_float("capture.rate_hz", 20.0)
    limiter = RateLimiter(rate)
    monitor = LoopMonitor()
    running = True

    def handle_signal(signum: int, frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    print(f"\nsessão: {session_dir}")
    print(f"controle: {source.name}   taxa alvo: {rate:.0f} Hz")
    if args.control == "rc":
        print("Pilote pelo rádio. O Jetson só grava.")
    print("Ctrl+C encerra e fecha a sessão corretamente.\n")

    if not args.no_arm:
        link.arm(True)

    recorder = SessionRecorder(session_dir, meta, config).start()
    mode = Mode.TELEOP if args.control != "rc" else Mode.IDLE

    try:
        while running:
            try:
                frame, t_host_ns = camera.read()
            except CameraError as exc:
                log.error("câmera falhou: %s", exc)
                break

            steer, throttle = source.read()
            if getattr(source, "quit_requested", False):
                break

            link.send_command(steer, throttle, mode)
            telemetry, age = link.telemetry_with_age()
            recorder.submit(
                frame, t_host_ns, telemetry, age, command=(steer, throttle)
            )

            monitor.tick()
            if recorder.stats.considered % 20 == 0:
                _print_status(recorder, monitor, telemetry)

            if args.max_frames and recorder.frames_written >= args.max_frames:
                break
            limiter.sleep()
    finally:
        # Ordem importa: primeiro para o carro, depois fecha o resto.
        link.stop()
        link.arm(False)
        source.close()
        camera.release()
        final = recorder.stop()  # drena a fila e grava session.json
        link.close()

    print("\n\n=== sessão encerrada ===")
    print(f"  diretório:        {session_dir}")
    print(f"  frames gravados:  {final.frame_count}")
    print(f"  duração:          {final.duration_s:.1f} s")
    print(f"  aproveitamento:   {final.yield_ratio * 100:.0f}%")
    print(f"  {recorder.stats.summary()}")
    print(f"  loop:             {monitor.summary()}")

    if final.frame_count == 0:
        print(
            "\nNENHUM frame gravado. Causas comuns:\n"
            "  - carro parado o tempo todo (filtro skip_when_stopped)\n"
            "  - telemetria ausente ou atrasada (confira `robocar link --watch`)\n"
            "  - ESP32 em failsafe (flag FAILSAFE ativa)"
        )
        return 1
    if final.yield_ratio < 0.5:
        print(
            f"\nAVISO: aproveitamento de {final.yield_ratio * 100:.0f}%. "
            "Veja quais contadores subiram acima e corrija antes da próxima sessão."
        )
    return 0


def _print_status(recorder, monitor, telemetry) -> None:
    steer = telemetry.steer if telemetry else 0.0
    throttle = telemetry.throttle if telemetry else 0.0
    print(
        f"  frames={recorder.frames_written:6d}  fila={recorder.queue_depth:3d}  "
        f"steer={steer:+.2f} thr={throttle:+.2f}  {monitor.summary()}",
        end="\r",
    )


def _prompt_metadata(config, args: argparse.Namespace) -> dict[str, str]:
    """Pergunta os metadados da sessão (ou usa o que veio por flag).

    Insistimos nisso porque uma pasta chamada ``20261204_143302`` sem contexto
    é praticamente inútil dois meses depois.
    """
    provided = {
        "track": args.track,
        "driver": args.driver,
        "lighting": args.lighting,
        "direction": args.direction,
        "notes": args.notes,
    }
    if args.no_prompt:
        return {k: v or "" for k, v in provided.items()}

    labels = {
        "track": "Pista (ex.: minicidade_fatec, garagem)",
        "driver": "Piloto",
        "lighting": "Iluminação (luz_natural, fluorescente, noturno)",
        "direction": "Sentido (horario, antihorario)",
        "notes": "Observações",
    }
    result = {}
    print("=== metadados da sessão ===")
    for key, label in labels.items():
        if provided[key]:
            result[key] = provided[key]
            print(f"{label}: {provided[key]}")
        else:
            try:
                result[key] = input(f"{label}: ").strip()
            except EOFError:
                result[key] = ""
    print()
    return result


# ---------------------------------------------------------------------------
# dataset
# ---------------------------------------------------------------------------


def cmd_dataset_stats(args: argparse.Namespace) -> int:
    """Resumo do que já foi coletado, por sessão e no total."""
    from .capture.schema import find_sessions, read_records, read_session_meta
    from .config import find_repo_root

    root = Path(args.root) if args.root else find_repo_root() / "data" / "sessions"
    sessions = find_sessions(root)
    if not sessions:
        print(f"nenhuma sessão em {root}")
        return 0

    total_frames = 0
    total_seconds = 0.0
    histogram = [0] * 9

    print(f"{'sessão':<42} {'frames':>7} {'dur(s)':>7} {'aprov':>6}  pista")
    print("-" * 90)
    for session_dir in sessions:
        meta = read_session_meta(session_dir)
        mark = "  [DESCARTADA]" if meta.discarded else ""
        print(
            f"{meta.session_id:<42} {meta.frame_count:>7} {meta.duration_s:>7.0f} "
            f"{meta.yield_ratio * 100:>5.0f}%  {meta.track}{mark}"
        )
        if meta.discarded:
            continue
        total_frames += meta.frame_count
        total_seconds += meta.duration_s
        if args.histogram:
            for record in read_records(session_dir):
                bucket = min(int((record.steer + 1.0) / 2.0 * 9), 8)
                histogram[bucket] += 1

    print("-" * 90)
    print(f"{'TOTAL':<42} {total_frames:>7} {total_seconds:>7.0f}")
    print(f"\n~{total_frames / 20 / 60:.1f} minutos de condução a 20 Hz")
    # ~280 kB por quadro JPEG q90 em 1640x1232 (ver docs/02-hardware.md).
    print(f"~{total_frames * KB_PER_FRAME / 1024**2:.1f} GB em disco (estimado)")

    if args.histogram and total_frames:
        print("\n=== distribuição do esterço ===")
        peak = max(histogram) or 1
        edges = [-1.0 + i * 2.0 / 9 for i in range(10)]
        for i, count in enumerate(histogram):
            bar = "#" * int(40 * count / peak)
            print(f"  [{edges[i]:+.2f}, {edges[i + 1]:+.2f}) {count:>7} {bar}")
        center = histogram[4] / total_frames
        if center > 0.5:
            print(
                f"\nATENÇÃO: {center * 100:.0f}% das amostras estão perto de esterço "
                "zero. Um modelo treinado assim tende a andar reto e não fazer as "
                "curvas. Ver docs/06-treinamento.md (balanceamento)."
            )
    return 0


def cmd_dataset_verify(args: argparse.Namespace) -> int:
    """Confere se cada registro tem imagem e se o esquema está íntegro."""
    from .capture.schema import (
        SchemaError,
        find_sessions,
        iter_records,
        read_session_meta,
    )
    from .config import find_repo_root

    root = Path(args.root) if args.root else find_repo_root() / "data" / "sessions"
    problems = 0

    for session_dir in find_sessions(root):
        meta = read_session_meta(session_dir)
        missing_images = 0
        bad_records = 0
        count = 0
        indices = set()
        duplicated = 0

        try:
            for record in iter_records(session_dir, strict=True):
                count += 1
                if record.index in indices:
                    duplicated += 1
                indices.add(record.index)
                if not (session_dir / record.image).exists():
                    missing_images += 1
        except SchemaError as exc:
            bad_records += 1
            print(f"  {session_dir.name}: registro inválido — {exc}")

        issues = []
        if missing_images:
            issues.append(f"{missing_images} imagens ausentes")
        if duplicated:
            issues.append(f"{duplicated} índices duplicados")
        if bad_records:
            issues.append(f"{bad_records} registros inválidos")
        if count != meta.frame_count:
            issues.append(f"session.json diz {meta.frame_count}, achei {count}")

        status = "ok" if not issues else "; ".join(issues)
        print(f"{'ok ' if not issues else 'ERRO'} {session_dir.name}: {count} registros — {status}")
        problems += len(issues)

    print(f"\n{problems} problema(s) encontrado(s).")
    return 0 if problems == 0 else 1


def cmd_dataset_tag(args: argparse.Namespace) -> int:
    """Marca uma sessão como descartada (sem apagar) ou edita observações."""
    from .capture.schema import read_session_meta, write_session_meta
    from .config import find_repo_root

    root = Path(args.root) if args.root else find_repo_root() / "data" / "sessions"
    session_dir = root / args.session
    if not session_dir.exists():
        print(f"sessão não encontrada: {session_dir}", file=sys.stderr)
        return 1

    meta = read_session_meta(session_dir)
    if args.discard is not None:
        meta.discarded = True
        meta.discard_reason = args.discard
        print(f"sessão marcada como descartada: {args.discard}")
    if args.keep:
        meta.discarded = False
        meta.discard_reason = ""
        print("sessão reativada")
    if args.notes:
        meta.notes = (meta.notes + " | " + args.notes).strip(" |")
        print(f"observações: {meta.notes}")

    write_session_meta(session_dir, meta)
    return 0


# ---------------------------------------------------------------------------
# calib
# ---------------------------------------------------------------------------


def cmd_calib_camera(args: argparse.Namespace) -> int:
    """Calibra os intrínsecos e a distorção da lente com um tabuleiro de xadrez.

    A lente de 120° tem distorção de barril forte. Calibrar permite construir a
    "câmera virtual" das placas (`robocar camera --virtual`), que entrega a
    placa retificada e sempre no mesmo enquadramento.
    """
    import cv2  # type: ignore[import-not-found]
    import numpy as np

    from .config import find_repo_root, load_config
    from .sensors.camera import CameraError, open_camera
    from .sensors.lens import CameraIntrinsics

    root = find_repo_root()
    config = load_config(root, profile=args.profile)

    try:
        cols, rows = (int(v) for v in args.chessboard.lower().split("x"))
    except ValueError:
        print(
            f"--chessboard inválido: {args.chessboard!r} (use algo como 9x6, "
            "contando CANTOS INTERNOS, não quadrados)",
            file=sys.stderr,
        )
        return 1

    print("=== calibração da lente ===")
    print(
        f"Imprima um tabuleiro de xadrez com {cols}x{rows} cantos internos e "
        f"quadrados de {args.square_mm} mm.\n"
        "Cole numa superfície RÍGIDA (papel ondulado arruína a calibração).\n"
    )
    print(
        "Capture ~20 imagens variando bastante:\n"
        "  - tabuleiro perto e longe\n"
        "  - inclinado para os 4 lados\n"
        "  - **nos CANTOS do quadro**, não só no centro — é lá que mora a\n"
        "    distorção de barril que queremos medir\n"
    )
    print("ESPAÇO captura  |  ENTER finaliza  |  Ctrl+C aborta\n")

    # Pontos 3D do tabuleiro no seu próprio referencial (z=0, plano).
    pattern = np.zeros((rows * cols, 3), np.float32)
    pattern[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    pattern *= args.square_mm

    object_points: list = []
    image_points: list = []
    shape: tuple[int, int] | None = None

    try:
        camera = open_camera(config, override_backend=args.backend)
    except CameraError as exc:
        print(f"erro de câmera: {exc}", file=sys.stderr)
        return 1

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    try:
        with camera:
            while True:
                frame, _ = camera.read()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                shape = gray.shape[::-1]

                found, corners = cv2.findChessboardCorners(
                    gray,
                    (cols, rows),
                    cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
                )
                status = "TABULEIRO OK " if found else "procurando... "
                print(
                    f"  {status} capturas={len(image_points):2d}  "
                    f"(ESPAÇO captura, ENTER finaliza)",
                    end="\r",
                )

                if args.show:
                    preview = frame.copy()
                    if found:
                        cv2.drawChessboardCorners(preview, (cols, rows), corners, found)
                    cv2.imshow("calibracao — ESPACO captura, ENTER finaliza", preview)
                    key = cv2.waitKey(1) & 0xFF
                else:
                    key = _read_key_nonblocking()

                if key in (13, 10):  # Enter
                    break
                if key == 32 and found:  # Espaço
                    refined = cv2.cornerSubPix(
                        gray, corners, (11, 11), (-1, -1), criteria
                    )
                    object_points.append(pattern.copy())
                    image_points.append(refined)
                    print(f"\n  capturada #{len(image_points)}")
    except KeyboardInterrupt:
        print("\nabortado.")
        return 1
    finally:
        if args.show:
            cv2.destroyAllWindows()

    if len(image_points) < 8:
        print(
            f"\nApenas {len(image_points)} capturas — insuficiente. "
            "Use pelo menos 8, idealmente 20.",
            file=sys.stderr,
        )
        return 1

    print(f"\ncalibrando com {len(image_points)} imagens...")
    rms, matrix, dist, _, _ = cv2.calibrateCamera(
        object_points, image_points, shape, None, None
    )

    intrinsics = CameraIntrinsics(
        fx=float(matrix[0, 0]),
        fy=float(matrix[1, 1]),
        cx=float(matrix[0, 2]),
        cy=float(matrix[1, 2]),
        width=int(shape[0]),
        height=int(shape[1]),
        dist=[float(d) for d in dist.ravel()[:5]],
        rms_error_px=float(rms),
        calibrated_at=_now_iso(),
        sample_count=len(image_points),
    )

    output = root / config.get_str(
        "camera.intrinsics_file", "config/calib/camera_intrinsics.yaml"
    )
    intrinsics.save(output)

    print("\n=== resultado ===")
    print(f"  erro de reprojeção (RMS): {rms:.3f} px")
    print(f"  HFOV medido:              {intrinsics.hfov_deg:.1f}°")
    print(f"  VFOV medido:              {intrinsics.vfov_deg:.1f}°")
    print(f"  DFOV medido:              {intrinsics.dfov_deg:.1f}°")
    print(f"  k1 (barril):              {intrinsics.dist[0]:+.4f}")
    print(f"  salvo em:                 {output}")

    if rms > 1.0:
        print(
            f"\nATENÇÃO: RMS de {rms:.2f} px é alto. Refaça com o tabuleiro em "
            "superfície rígida, boa iluminação e mais poses nos cantos do quadro."
        )
    declared = config.get_float("camera.hardware.fov_horizontal_deg", 0.0)
    if declared and abs(declared - intrinsics.hfov_deg) > 8:
        print(
            f"\nO HFOV medido ({intrinsics.hfov_deg:.0f}°) difere bastante do "
            f"declarado em config/camera.yaml ({declared:.0f}°). "
            f"Atualize `camera.hardware.fov_horizontal_deg` — as contas de "
            f"viabilidade das placas dependem desse número."
        )
    return 0


def _read_key_nonblocking() -> int:
    """Lê uma tecla sem bloquear, para o modo sem janela gráfica."""
    import select as _select
    import sys as _sys

    if not _sys.stdin.isatty():
        return -1
    if _select.select([_sys.stdin], [], [], 0.01)[0]:
        return ord(_sys.stdin.read(1))
    return -1


def _now_iso() -> str:
    from datetime import datetime

    return datetime.now().astimezone().isoformat(timespec="seconds")


def cmd_calib_steering(args: argparse.Namespace) -> int:
    """Assistente de calibração da direção. **Rode com as rodas no ar.**"""
    from .comms.link import LinkError, SerialLink
    from .comms.protocol import Mode, encode_config

    print("=== calibração da direção ===")
    print("ATENÇÃO: coloque o carro no cavalete, com as rodas fora do chão.\n")
    try:
        input("Pressione Enter para continuar (Ctrl+C para abortar)... ")
    except (EOFError, KeyboardInterrupt):
        return 1

    try:
        with SerialLink(args.port, args.baud) as link:
            if link.ping(timeout=2.0) is None:
                print("ESP32 não respondeu.", file=sys.stderr)
                return 1

            values = {}
            for label, key, start in (
                ("CENTRO (rodas retas)", "steer_center_us", 1500),
                ("ESQUERDA máxima", "steer_min_us", 1200),
                ("DIREITA máxima", "steer_max_us", 1800),
            ):
                current = start
                print(f"\n--- {label} ---")
                print("  'a'/'d' ajusta 10 us, 'A'/'D' ajusta 50 us, Enter confirma.")
                while True:
                    link.send_raw(encode_config("steer_center_us", current))
                    link.send_command(0.0, 0.0, Mode.IDLE)
                    try:
                        keys = input(f"  {current} us > ").strip()
                    except (EOFError, KeyboardInterrupt):
                        return 1
                    if not keys:
                        break
                    for char in keys:
                        current += {"a": -10, "d": 10, "A": -50, "D": 50}.get(char, 0)
                    current = max(800, min(2400, current))
                values[key] = current
                print(f"  {key} = {current}")

            print("\n=== copie para config/vehicle.yaml, seção `steering` ===")
            print(f"  center_us: {values['steer_center_us']}")
            print(f"  min_us: {values['steer_min_us']}")
            print(f"  max_us: {values['steer_max_us']}")
            if not values["steer_min_us"] < values["steer_center_us"] < values["steer_max_us"]:
                print(
                    "\nAVISO: o centro não ficou entre os batentes. Se a direção "
                    "responde invertida, ajuste `steering.invert: true` e refaça."
                )
    except LinkError as exc:
        print(f"erro de enlace: {exc}", file=sys.stderr)
        return 1
    return 0


# ---------------------------------------------------------------------------
# parser
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="robocar",
        description="Ferramentas do carro autônomo C.A.FATEC.R.B — RoboCar Race 2026",
    )
    parser.add_argument("--version", action="version", version=f"robocar {__version__}")
    parser.add_argument("-v", "--verbose", action="store_true", help="log detalhado")
    parser.add_argument(
        "--profile", default=None, help="perfil de config/profiles/ (ex.: race)"
    )
    sub = parser.add_subparsers(dest="command", required=True)

    # doctor
    doctor = sub.add_parser("doctor", help="confere o ambiente")
    doctor.set_defaults(func=cmd_doctor)

    # link
    link = sub.add_parser("link", help="testa o enlace com o ESP32")
    link.add_argument("--port", default=None, help="porta serial (autodetecta)")
    link.add_argument("--baud", type=int, default=115200)
    link.add_argument("--watch", action="store_true", help="mostra telemetria contínua")
    link.add_argument("--handshake", action="store_true", help="envia a configuração")
    link.add_argument("--rc", action="store_true", help="habilita o rádio RC")
    link.set_defaults(func=cmd_link)

    # camera
    camera = sub.add_parser("camera", help="testa a câmera")
    camera.add_argument("--backend", choices=["csi", "v4l2", "file"], default=None)
    camera.add_argument("--frames", type=int, default=0, help="0 = até Ctrl+C")
    camera.add_argument("--snapshot", default=None, help="salva o primeiro quadro")
    camera.add_argument(
        "--focus",
        action="store_true",
        help="medidor de nitidez ao vivo, para ajustar a rosca de foco da lente",
    )
    camera.add_argument(
        "--virtual",
        action="store_true",
        help="também gera a vista virtual retificada das placas (precisa de calibração)",
    )
    camera.set_defaults(func=cmd_camera)

    # record
    record = sub.add_parser("record", help="grava uma sessão de dataset")
    record.add_argument(
        "--control",
        choices=["rc", "gamepad", "keyboard"],
        default="rc",
        help="quem pilota durante a coleta (padrão: rádio RC)",
    )
    record.add_argument("--port", default=None)
    record.add_argument("--baud", type=int, default=115200)
    record.add_argument("--backend", choices=["csi", "v4l2", "file"], default=None)
    record.add_argument("--tag", default="", help="sufixo do nome da sessão")
    record.add_argument("--track", default="")
    record.add_argument("--driver", default="")
    record.add_argument("--lighting", default="")
    record.add_argument("--direction", default="")
    record.add_argument("--notes", default="")
    record.add_argument("--no-prompt", action="store_true", help="não pergunta metadados")
    record.add_argument("--no-arm", action="store_true", help="não arma a tração")
    record.add_argument("--max-frames", type=int, default=0, help="0 = ilimitado")
    record.set_defaults(func=cmd_record)

    # dataset
    dataset = sub.add_parser("dataset", help="inspeciona o dataset coletado")
    dataset_sub = dataset.add_subparsers(dest="subcommand", required=True)

    stats = dataset_sub.add_parser("stats", help="resumo das sessões")
    stats.add_argument("--root", default=None)
    stats.add_argument(
        "--histogram", action="store_true", help="distribuição do esterço"
    )
    stats.set_defaults(func=cmd_dataset_stats)

    verify = dataset_sub.add_parser("verify", help="confere integridade")
    verify.add_argument("--root", default=None)
    verify.set_defaults(func=cmd_dataset_verify)

    tag = dataset_sub.add_parser("tag", help="marca ou anota uma sessão")
    tag.add_argument("session", help="nome do diretório da sessão")
    tag.add_argument("--root", default=None)
    tag.add_argument("--discard", default=None, metavar="MOTIVO")
    tag.add_argument("--keep", action="store_true", help="desfaz o descarte")
    tag.add_argument("--notes", default=None)
    tag.set_defaults(func=cmd_dataset_tag)

    # calib
    calib = sub.add_parser("calib", help="assistentes de calibração")
    calib_sub = calib.add_subparsers(dest="subcommand", required=True)
    steering = calib_sub.add_parser("steering", help="centro e batentes da direção")
    steering.add_argument("--port", default=None)
    steering.add_argument("--baud", type=int, default=115200)
    steering.set_defaults(func=cmd_calib_steering)

    calib_camera = calib_sub.add_parser(
        "camera", help="intrínsecos e distorção da lente (tabuleiro de xadrez)"
    )
    calib_camera.add_argument(
        "--chessboard",
        default="9x6",
        help="cantos INTERNOS do tabuleiro, ex.: 9x6 (padrão: 9x6)",
    )
    calib_camera.add_argument(
        "--square-mm", type=float, default=25.0, help="lado do quadrado em mm"
    )
    calib_camera.add_argument("--backend", choices=["csi", "v4l2", "file"], default=None)
    calib_camera.add_argument(
        "--show", action="store_true", help="abre janela com a prévia (precisa de display)"
    )
    calib_camera.set_defaults(func=cmd_calib_camera)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    _setup_logging(args.verbose)
    try:
        return int(args.func(args))
    except KeyboardInterrupt:
        print("\ninterrompido.")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
