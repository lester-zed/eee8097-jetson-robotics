#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
import sys
import time

import cv2


DEFAULT_OUTPUT = Path("/workspace/data/tissue_pack/raw")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture tissue-pack YOLO dataset images from the Jetson USB camera."
    )
    parser.add_argument("--camera", default="0")
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--jpeg-quality", type=int, default=95)
    parser.add_argument("--warmup-frames", type=int, default=15)
    parser.add_argument("--no-preview", action="store_true")
    parser.add_argument(
        "--auto",
        choices=("positive", "negative"),
        default=None,
        help="Automatically capture positive or negative samples.",
    )
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--count", type=int, default=0)
    return parser.parse_args()


def camera_source(value: str) -> int | str:
    stripped = str(value).strip()
    return int(stripped) if stripped.isdigit() else stripped


def open_camera(source: int | str) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
    if cap.isOpened():
        return cap
    cap.release()
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f"Could not open camera source: {source!r}")
    return cap


def ensure_directories(output_root: Path) -> tuple[Path, Path]:
    positive = output_root / "positive"
    negative = output_root / "negative"
    positive.mkdir(parents=True, exist_ok=True)
    negative.mkdir(parents=True, exist_ok=True)
    return positive, negative


def existing_count(directory: Path) -> int:
    return sum(1 for p in directory.glob("*.jpg") if p.is_file())


def save_frame(
    frame,
    *,
    directory: Path,
    prefix: str,
    sequence: int,
    jpeg_quality: int,
    manifest_path: Path,
    label: str,
    camera: str,
) -> Path:
    now = datetime.now()
    timestamp = now.strftime("%Y%m%d_%H%M%S_%f")[:-3]
    path = directory / f"{prefix}_{timestamp}_{sequence:06d}.jpg"

    ok = cv2.imwrite(
        str(path),
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)],
    )
    if not ok:
        raise RuntimeError(f"Failed to write image: {path}")

    height, width = frame.shape[:2]
    new_manifest = not manifest_path.exists()
    with manifest_path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        if new_manifest:
            writer.writerow(
                [
                    "filename",
                    "label_hint",
                    "captured_at_local",
                    "width",
                    "height",
                    "camera_source",
                ]
            )
        writer.writerow(
            [
                str(path.relative_to(manifest_path.parent)),
                label,
                now.isoformat(timespec="milliseconds"),
                width,
                height,
                camera,
            ]
        )
    return path


def overlay(frame, positive_count: int, negative_count: int, auto_mode: str | None) -> None:
    instruction = (
        f"AUTO={auto_mode} | q/ESC quit"
        if auto_mode
        else "SPACE/p: tissue_pack | n: negative | q/ESC: quit"
    )
    cv2.putText(
        frame,
        instruction,
        (15, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 0),
        2,
    )
    cv2.putText(
        frame,
        f"positive={positive_count}  negative={negative_count}",
        (15, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2,
    )


def main() -> int:
    args = parse_args()

    if not 1 <= args.jpeg_quality <= 100:
        raise ValueError("--jpeg-quality must be in [1, 100]")
    if args.width <= 0 or args.height <= 0:
        raise ValueError("--width and --height must be positive")
    if args.warmup_frames < 0:
        raise ValueError("--warmup-frames cannot be negative")
    if args.interval <= 0.0:
        raise ValueError("--interval must be positive")
    if args.count < 0:
        raise ValueError("--count cannot be negative")
    if args.no_preview and args.auto is None:
        raise ValueError("--no-preview requires --auto positive|negative")

    output_root = Path(args.output).expanduser()
    positive_dir, negative_dir = ensure_directories(output_root)
    manifest_path = output_root / "capture_manifest.csv"

    source = camera_source(args.camera)
    cap = open_camera(source)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    print("=" * 72)
    print("EEE8097 tissue_pack dataset capture")
    print(f"Camera: {source}")
    print(f"Output: {output_root}")
    print("=" * 72)

    try:
        for _ in range(args.warmup_frames):
            ok, _ = cap.read()
            if not ok:
                raise RuntimeError("Camera failed during warm-up")

        positive_count = existing_count(positive_dir)
        negative_count = existing_count(negative_dir)
        auto_captured = 0
        next_auto_at = time.monotonic()

        while True:
            ok, frame = cap.read()
            if not ok or frame is None or getattr(frame, "size", 0) == 0:
                raise RuntimeError("Camera returned an empty frame")

            if args.auto is not None and time.monotonic() >= next_auto_at:
                if args.auto == "positive":
                    positive_count += 1
                    path = save_frame(
                        frame,
                        directory=positive_dir,
                        prefix="tissue",
                        sequence=positive_count,
                        jpeg_quality=args.jpeg_quality,
                        manifest_path=manifest_path,
                        label="tissue_pack",
                        camera=str(source),
                    )
                else:
                    negative_count += 1
                    path = save_frame(
                        frame,
                        directory=negative_dir,
                        prefix="negative",
                        sequence=negative_count,
                        jpeg_quality=args.jpeg_quality,
                        manifest_path=manifest_path,
                        label="negative",
                        camera=str(source),
                    )
                auto_captured += 1
                print(f"SAVED [{auto_captured}]: {path}")
                next_auto_at = time.monotonic() + args.interval
                if args.count and auto_captured >= args.count:
                    break

            if args.no_preview:
                time.sleep(0.01)
                continue

            display_frame = frame.copy()
            overlay(display_frame, positive_count, negative_count, args.auto)
            cv2.imshow("EEE8097 tissue_pack dataset capture", display_frame)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break
            if key in (ord("p"), ord(" ")):
                positive_count += 1
                path = save_frame(
                    frame,
                    directory=positive_dir,
                    prefix="tissue",
                    sequence=positive_count,
                    jpeg_quality=args.jpeg_quality,
                    manifest_path=manifest_path,
                    label="tissue_pack",
                    camera=str(source),
                )
                print(f"SAVED POSITIVE: {path}")
            elif key == ord("n"):
                negative_count += 1
                path = save_frame(
                    frame,
                    directory=negative_dir,
                    prefix="negative",
                    sequence=negative_count,
                    jpeg_quality=args.jpeg_quality,
                    manifest_path=manifest_path,
                    label="negative",
                    camera=str(source),
                )
                print(f"SAVED NEGATIVE: {path}")

        print(f"Capture complete. positive={positive_count}, negative={negative_count}")
        print(f"Manifest: {manifest_path}")
        return 0
    finally:
        cap.release()
        if not args.no_preview:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nCapture interrupted.", file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(f"FAIL: {type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(2)
