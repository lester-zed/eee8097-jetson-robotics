#!/usr/bin/env python3
"""
check_docker_env.py

A small diagnostic script for checking a Jetson Docker development environment.

It checks:
- Python version and executable
- OS / architecture
- important environment variables
- mounted workspace paths
- common Python packages
- optional PyTorch / CUDA availability
- OpenCV availability
- camera / serial device visibility
- NVIDIA / Jetson-related commands if available

Usage inside container:
    python3 check_docker_env.py

Recommended:
    python3 check_docker_env.py | tee logs/docker_env_check.txt
"""

from __future__ import annotations

import importlib
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Iterable


def print_section(title: str) -> None:
    print("\n" + "=" * 80)
    print(f"[ {title} ]")
    print("=" * 80)


def run_cmd(cmd: list[str], timeout: int = 5) -> tuple[int, str]:
    try:
        result = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
            check=False,
        )
        return result.returncode, result.stdout.strip()
    except FileNotFoundError:
        return 127, f"Command not found: {cmd[0]}"
    except subprocess.TimeoutExpired:
        return 124, f"Command timed out: {' '.join(cmd)}"
    except Exception as exc:
        return 1, f"Command failed: {exc}"


def check_path(path: str) -> None:
    p = Path(path)
    if p.exists():
        if p.is_dir():
            print(f"[OK]   {path:<30} exists, directory")
        elif p.is_file():
            print(f"[OK]   {path:<30} exists, file")
        else:
            print(f"[OK]   {path:<30} exists")
    else:
        print(f"[MISS] {path:<30} not found")


def check_import(module_name: str, import_name: str | None = None) -> None:
    name = import_name or module_name
    try:
        module = importlib.import_module(name)
        version = getattr(module, "__version__", "unknown")
        print(f"[OK]   import {module_name:<18} version: {version}")
    except Exception as exc:
        print(f"[FAIL] import {module_name:<18} error: {exc}")


def list_matching_devices(prefixes: Iterable[str]) -> list[str]:
    found: list[str] = []
    for prefix in prefixes:
        found.extend(str(p) for p in sorted(Path("/dev").glob(prefix)))
    return found


def main() -> int:
    print_section("Basic Python Environment")
    print(f"Python executable : {sys.executable}")
    print(f"Python version    : {sys.version.replace(os.linesep, ' ')}")
    print(f"Working directory : {Path.cwd()}")
    print(f"User              : UID={os.getuid()} GID={os.getgid()}")
    print(f"Platform          : {platform.platform()}")
    print(f"Machine           : {platform.machine()}")
    print(f"Processor         : {platform.processor()}")

    print_section("Container / NVIDIA Environment Variables")
    env_keys = [
        "PATH",
        "PYTHONPATH",
        "VIRTUAL_ENV",
        "DISPLAY",
        "NVIDIA_VISIBLE_DEVICES",
        "NVIDIA_DRIVER_CAPABILITIES",
        "LD_LIBRARY_PATH",
        "CUDA_HOME",
        "CUDA_PATH",
        "ROS_DISTRO",
    ]
    for key in env_keys:
        value = os.environ.get(key)
        if value:
            print(f"{key:<28}= {value}")
        else:
            print(f"{key:<28}= <not set>")

    print_section("Important Paths")
    for path in [
        "/workspace",
        "/workspace/src",
        "/workspace/src/configs",
        "/workspace/data",
        "/workspace/models",
        "/workspace/logs",
        "/dev",
        "/tmp/.X11-unix",
        "/usr/local/cuda",
    ]:
        check_path(path)

    print_section("Python Package Imports")
    packages = [
        ("numpy", None),
        ("yaml", "yaml"),
        ("matplotlib", None),
        ("cv2", "cv2"),
        ("torch", None),
        ("PIL", "PIL"),
        ("serial", "serial"),
    ]
    for module_name, import_name in packages:
        check_import(module_name, import_name)

    print_section("OpenCV Details")
    try:
        import cv2  # type: ignore

        print(f"[OK] OpenCV version: {cv2.__version__}")
        build_info = cv2.getBuildInformation()
        interesting_keywords = [
            "GStreamer",
            "FFMPEG",
            "CUDA",
            "V4L/V4L2",
            "Python",
        ]
        for line in build_info.splitlines():
            if any(k in line for k in interesting_keywords):
                print(line)
    except Exception as exc:
        print(f"[FAIL] OpenCV detail check failed: {exc}")

    print_section("PyTorch / CUDA Check")
    try:
        import torch  # type: ignore

        print(f"[OK] torch version        : {torch.__version__}")
        print(f"[INFO] cuda available    : {torch.cuda.is_available()}")
        print(f"[INFO] cuda device count : {torch.cuda.device_count()}")
        if torch.cuda.is_available():
            print(f"[INFO] cuda device name  : {torch.cuda.get_device_name(0)}")
            x = torch.randn(256, 256, device="cuda")
            y = x @ x
            print(f"[OK] CUDA tensor test    : mean={y.mean().item():.6f}")
        else:
            print("[WARN] PyTorch is installed but CUDA is not available.")
            print("       This may be normal if you have not installed a Jetson-compatible PyTorch container/package.")
    except Exception as exc:
        print(f"[WARN] PyTorch check skipped or failed: {exc}")

    print_section("Device Visibility")
    video_devices = list_matching_devices(["video*"])
    serial_devices = list_matching_devices(["ttyUSB*", "ttyACM*"])
    i2c_devices = list_matching_devices(["i2c-*"])

    if video_devices:
        print("[OK] Video devices:")
        for d in video_devices:
            print(f"     {d}")
    else:
        print("[INFO] No /dev/video* devices found.")

    if serial_devices:
        print("[OK] Serial devices:")
        for d in serial_devices:
            print(f"     {d}")
    else:
        print("[INFO] No /dev/ttyUSB* or /dev/ttyACM* devices found.")

    if i2c_devices:
        print("[OK] I2C devices:")
        for d in i2c_devices:
            print(f"     {d}")
    else:
        print("[INFO] No /dev/i2c-* devices found.")

    print_section("System Commands")
    commands = [
        ["uname", "-a"],
        ["lsb_release", "-a"],
        ["cat", "/etc/os-release"],
        ["df", "-h"],
        ["free", "-h"],
    ]

    for cmd in commands:
        rc, out = run_cmd(cmd)
        print(f"\n$ {' '.join(cmd)}")
        if rc == 0:
            print(out)
        else:
            print(f"[WARN] return code {rc}: {out}")

    print_section("NVIDIA / Jetson Commands")
    nvidia_commands = [
        ["tegrastats", "--help"],
        ["nvpmodel", "-q"],
        ["nvcc", "--version"],
    ]

    for cmd in nvidia_commands:
        print(f"\n$ {' '.join(cmd)}")
        if shutil.which(cmd[0]) is None:
            print(f"[INFO] {cmd[0]} not found inside this container.")
            continue

        rc, out = run_cmd(cmd, timeout=5)
        if rc == 0:
            # Avoid printing too much help text.
            lines = out.splitlines()
            if len(lines) > 20:
                print("\n".join(lines[:20]))
                print("... <truncated>")
            else:
                print(out)
        else:
            print(f"[WARN] return code {rc}: {out}")

    print_section("Summary / What To Look For")
    print("""
1. If /workspace/src, /workspace/data, /workspace/logs exist:
   -> Your Docker volume mounting is likely working.

2. If cv2 can be imported:
   -> Basic OpenCV Python environment is available.

3. If /dev/video* appears:
   -> Camera device is visible inside the container.
      If not, it may be because no camera is connected, or the container did not map the device.

4. If /dev/ttyUSB* or /dev/ttyACM* appears:
   -> USB serial devices such as manipulator arms may be visible inside the container.

5. If torch.cuda.is_available() is False:
   -> This does not necessarily mean Docker is broken.
      It usually means PyTorch with Jetson CUDA support is not installed yet.

6. If nvpmodel / tegrastats are missing inside the container:
   -> That can be normal. You can run them on the Jetson host instead.
""".strip())

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

