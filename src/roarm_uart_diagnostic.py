#!/usr/bin/env python3
"""Read-only UART diagnostic for a Waveshare RoArm-M2-S.

This file is intentionally standalone and additive-only.  It does not import or
modify the project's existing RoArm controller.  The only command it can send is
the official read-only status request::

    {"T":105}\n

It never sends a motion, torque, LED, gripper, or reset command.

Typical Docker usage (from /workspace/src)::

    python roarm_uart_diagnostic.py --self-test
    python roarm_uart_diagnostic.py --port /dev/ttyROARM

The hardware run prints a concise verdict and writes a detailed JSON report to
``/tmp/roarm_uart_diagnostic_report.json`` by default.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import errno
import glob
import grp
import json
import os
from pathlib import Path
import pwd
import stat
import sys
import time
from typing import Any, Iterable


SCRIPT_VERSION = "1.0.0"
STATUS_REQUEST = b'{"T":105}\n'
DEFAULT_REPORT = "/tmp/roarm_uart_diagnostic_report.json"
SYSFS_USB_FIELDS = (
    "idVendor",
    "idProduct",
    "manufacturer",
    "product",
    "serial",
    "bInterfaceNumber",
    "interface",
)


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_text(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8", errors="replace").strip()
    except (OSError, PermissionError):
        return None


def json_safe(value: Any) -> Any:
    """Convert values used in the report to JSON-compatible structures."""
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [json_safe(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, bytes):
        return {
            "utf8": value.decode("utf-8", errors="replace"),
            "hex": value.hex(" "),
        }
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return repr(value)


def extract_json_objects(raw: bytes) -> list[dict[str, Any]]:
    """Extract JSON objects even when boot text or missing newlines surround them."""
    text = raw.decode("utf-8", errors="replace")
    decoder = json.JSONDecoder()
    output: list[dict[str, Any]] = []
    cursor = 0

    while cursor < len(text):
        start = text.find("{", cursor)
        if start < 0:
            break
        try:
            value, consumed = decoder.raw_decode(text[start:])
        except json.JSONDecodeError:
            cursor = start + 1
            continue
        if isinstance(value, dict):
            output.append(value)
        cursor = start + max(consumed, 1)
    return output


def payload_type(payload: dict[str, Any]) -> int | None:
    try:
        return int(payload.get("T"))
    except (TypeError, ValueError):
        return None


def has_t1051(payloads: Iterable[dict[str, Any]]) -> bool:
    return any(payload_type(payload) == 1051 for payload in payloads)


def process_identity() -> dict[str, Any]:
    uid = os.geteuid()
    gid = os.getegid()
    try:
        user = pwd.getpwuid(uid).pw_name
    except KeyError:
        user = str(uid)
    try:
        group = grp.getgrgid(gid).gr_name
    except KeyError:
        group = str(gid)

    groups: list[str] = []
    for item in os.getgroups():
        try:
            groups.append(grp.getgrgid(item).gr_name)
        except KeyError:
            groups.append(str(item))
    return {
        "pid": os.getpid(),
        "uid": uid,
        "user": user,
        "gid": gid,
        "primary_group": group,
        "supplementary_groups": sorted(set(groups)),
    }


def sysfs_metadata_for_device(device_path: str) -> dict[str, Any]:
    """Resolve USB identity through the device node's major/minor numbers.

    This still works when Docker maps host /dev/ttyUSB0 to a differently named
    container node such as /dev/ttyROARM.
    """
    result: dict[str, Any] = {}
    try:
        node_stat = os.stat(device_path)
    except OSError as exc:
        return {"error": f"{type(exc).__name__}: {exc}"}

    if not stat.S_ISCHR(node_stat.st_mode):
        return {"error": "path is not a character device"}

    major = os.major(node_stat.st_rdev)
    minor = os.minor(node_stat.st_rdev)
    result["major"] = major
    result["minor"] = minor
    sys_char = Path(f"/sys/dev/char/{major}:{minor}")
    result["sys_char_path"] = str(sys_char)

    try:
        resolved = sys_char.resolve(strict=True)
    except OSError as exc:
        result["sysfs_error"] = f"{type(exc).__name__}: {exc}"
        return result

    result["resolved_sysfs_path"] = str(resolved)
    result["kernel_tty_name"] = resolved.name

    attributes: dict[str, str] = {}
    visited: list[str] = []
    current = resolved
    for _ in range(10):
        visited.append(str(current))
        for field in SYSFS_USB_FIELDS:
            value = read_text(current / field)
            if value and field not in attributes:
                attributes[field] = value
        if current.parent == current:
            break
        current = current.parent
    result["usb_attributes"] = attributes
    result["sysfs_ancestors_checked"] = visited
    return result


def device_summary(device_path: str) -> dict[str, Any]:
    item: dict[str, Any] = {
        "path": device_path,
        "lexists": os.path.lexists(device_path),
        "resolved_path": os.path.realpath(device_path),
        "readable": os.access(device_path, os.R_OK),
        "writable": os.access(device_path, os.W_OK),
    }
    try:
        node_stat = os.stat(device_path)
        item.update(
            {
                "is_character_device": stat.S_ISCHR(node_stat.st_mode),
                "mode_octal": oct(stat.S_IMODE(node_stat.st_mode)),
                "owner_uid": node_stat.st_uid,
                "owner_gid": node_stat.st_gid,
                "major": os.major(node_stat.st_rdev),
                "minor": os.minor(node_stat.st_rdev),
            }
        )
        try:
            item["owner_user"] = pwd.getpwuid(node_stat.st_uid).pw_name
        except KeyError:
            item["owner_user"] = str(node_stat.st_uid)
        try:
            item["owner_group"] = grp.getgrgid(node_stat.st_gid).gr_name
        except KeyError:
            item["owner_group"] = str(node_stat.st_gid)
        item["sysfs"] = sysfs_metadata_for_device(device_path)
    except OSError as exc:
        item["stat_error"] = f"{type(exc).__name__}: {exc}"
    return item


def serial_candidates() -> list[dict[str, Any]]:
    patterns = (
        "/dev/ttyROARM",
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
        "/dev/serial/by-id/*",
    )
    paths: set[str] = set()
    for pattern in patterns:
        paths.update(glob.glob(pattern))
    return [device_summary(path) for path in sorted(paths)]


def visible_processes_using(device_path: str) -> list[dict[str, Any]]:
    """Find matching open file descriptors visible in this PID namespace."""
    try:
        target_stat = os.stat(device_path)
    except OSError:
        return []

    output: list[dict[str, Any]] = []
    own_pid = os.getpid()
    for proc_dir_text in glob.glob("/proc/[0-9]*"):
        proc_dir = Path(proc_dir_text)
        try:
            pid = int(proc_dir.name)
        except ValueError:
            continue
        if pid == own_pid:
            continue

        matching_fds: list[int] = []
        for fd_text in glob.glob(str(proc_dir / "fd" / "*")):
            try:
                fd_stat = os.stat(fd_text)
            except OSError:
                continue
            if stat.S_ISCHR(fd_stat.st_mode) and fd_stat.st_rdev == target_stat.st_rdev:
                try:
                    matching_fds.append(int(Path(fd_text).name))
                except ValueError:
                    pass

        if not matching_fds:
            continue
        cmdline_raw = read_text(proc_dir / "cmdline") or ""
        command = cmdline_raw.replace("\x00", " ").strip()
        output.append(
            {
                "pid": pid,
                "fds": matching_fds,
                "command": command or read_text(proc_dir / "comm") or "unknown",
            }
        )
    return output


def read_for_seconds(serial_port: Any, seconds: float, max_bytes: int) -> bytes:
    deadline = time.monotonic() + seconds
    output = bytearray()

    while time.monotonic() < deadline and len(output) < max_bytes:
        try:
            waiting = int(getattr(serial_port, "in_waiting", 0) or 0)
        except (OSError, TypeError, ValueError):
            waiting = 0
        read_size = min(max(waiting, 1), max_bytes - len(output))
        chunk = serial_port.read(read_size)
        if chunk:
            output.extend(chunk)
            if has_t1051(extract_json_objects(bytes(output))):
                break
    return bytes(output)


def classify_report(report: dict[str, Any]) -> dict[str, Any]:
    serial_result = report.get("serial", {})
    attempts = serial_result.get("attempts", [])
    all_raw = b"".join(item.get("raw_bytes_internal", b"") for item in attempts)
    payloads: list[dict[str, Any]] = []
    for item in attempts:
        payloads.extend(item.get("json_payloads", []))

    device = report.get("selected_device", {})
    other_users = report.get("visible_processes_using_port", [])
    open_error = serial_result.get("open_error")
    usb_attrs = device.get("sysfs", {}).get("usb_attributes", {})
    usb_identity = " ".join(str(value) for value in usb_attrs.values()).lower()

    if open_error:
        error_number = serial_result.get("open_errno")
        if str(open_error).startswith("pyserial unavailable:"):
            summary = "FAIL：当前 Python 环境缺少 pyserial。"
            next_steps = [
                "请在运行该项目的 Docker 环境中执行；该环境应能 import serial。"
            ]
        elif error_number in (errno.EACCES, errno.EPERM):
            summary = "FAIL：容器内没有该串口的读写权限。"
            next_steps = [
                "检查 Docker 的 --device 映射以及容器用户的 dialout 组。",
                "在容器内执行 ls -l <port> 和 id 后重新运行。",
            ]
        elif error_number in (errno.EBUSY, errno.EAGAIN) or other_users:
            summary = "FAIL：串口正在被另一个进程占用。"
            next_steps = [
                "停止报告中列出的串口读取进程，然后重新运行本脚本。",
                "不要同时运行原 RoArm CLI、厂家串口 Demo 或 ROS2 驱动。",
            ]
        else:
            summary = "FAIL：串口节点存在，但无法打开。"
            next_steps = ["根据 serial.open_error 修复端口映射或设备连接后重试。"]
        return {
            "status": "FAIL",
            "exit_code": 3,
            "summary_zh": summary,
            "likely_causes_zh": [open_error],
            "next_steps_zh": next_steps,
        }

    if has_t1051(payloads):
        return {
            "status": "PASS",
            "exit_code": 0,
            "summary_zh": "PASS：已收到官方 T=1051 状态反馈，RoArm UART 通信正常。",
            "likely_causes_zh": [],
            "next_steps_zh": [
                "原 CLI 若仍偶发超时，优先把启动稳定时间和响应超时调长；不要先改运动逻辑。"
            ],
        }

    if not all_raw:
        likely = []
        if "slamtec" in usb_identity or "rplidar" in usb_identity:
            likely.append(
                "所选端口的 USB 元数据显示为 Slamtec/RPLIDAR；/dev/ttyROARM 映射到了错误设备。"
            )
        likely.extend(
            [
                "Docker 的 /dev/ttyROARM 可能映射到了另一台 ttyUSB 设备（你同时连接了 RPLIDAR）。",
                "RoArm 控制板未正常供电、USB 线只有供电无数据，或控制板固件未响应 UART。",
                "另一个宿主机进程可能已打开并读走反馈；容器内 /proc 不一定能看到宿主机进程。",
            ]
        )
        return {
            "status": "FAIL",
            "exit_code": 4,
            "summary_zh": "FAIL：三次只读 T=105 请求均收到 0 字节；这不是 T=1051 JSON 解析问题。",
            "likely_causes_zh": likely,
            "next_steps_zh": [
                "先依据报告 selected_device.sysfs.usb_attributes 核对 /dev/ttyROARM 的真实 USB 产品。",
                "在 Jetson 宿主机停止其他串口程序，并用 /dev/serial/by-id 建立稳定设备映射。",
                "确认 RoArm 单独供电正常，再拔插 USB 并重新创建包含正确 --device 的容器。",
            ],
        }

    if payloads:
        received_types = sorted(
            {item for item in (payload_type(payload) for payload in payloads) if item is not None}
        )
        return {
            "status": "FAIL",
            "exit_code": 5,
            "summary_zh": "FAIL：串口有 JSON 返回，但没有官方 T=1051 状态反馈。",
            "likely_causes_zh": [
                f"收到的 T 类型为 {received_types or 'unknown'}；可能是固件/协议版本差异或错误串口设备。"
            ],
            "next_steps_zh": [
                "查看 attempts[].json_payloads，不要在确认消息含义前发送任何运动指令。"
            ],
        }

    return {
        "status": "FAIL",
        "exit_code": 6,
        "summary_zh": "FAIL：串口有原始字节返回，但不是可解析的 JSON。",
        "likely_causes_zh": [
            "可能选择了错误设备、波特率不匹配，或只收到了控制板启动/异常文本。"
        ],
        "next_steps_zh": [
            "查看 attempts[].raw_utf8 和 raw_hex，并核对 USB 产品及 115200 8N1 设置。"
        ],
    }


def run_diagnostic(args: argparse.Namespace) -> tuple[dict[str, Any], int]:
    report: dict[str, Any] = {
        "schema_version": 1,
        "script_version": SCRIPT_VERSION,
        "started_at_utc": now_iso(),
        "safety": {
            "physical_motion_commands_sent": False,
            "only_transmit_frame_utf8": STATUS_REQUEST.decode("ascii").rstrip("\n"),
            "only_transmit_frame_hex": STATUS_REQUEST.hex(" "),
            "meaning": "Official read-only RoArm status request (T=105)",
        },
        "request": {
            "port": args.port,
            "baudrate": args.baudrate,
            "attempts": args.attempts,
            "settle_seconds": args.settle_seconds,
            "response_timeout_seconds": args.response_timeout,
            "max_rx_bytes_per_attempt": args.max_rx_bytes,
        },
        "runtime": {
            "python": sys.version,
            "platform": sys.platform,
            "process": process_identity(),
        },
        "selected_device": device_summary(args.port),
        "visible_serial_candidates": serial_candidates(),
        "visible_processes_using_port": visible_processes_using(args.port),
        "serial": {"attempts": [], "warnings": []},
    }

    selected = report["selected_device"]
    if not selected.get("lexists") or not selected.get("is_character_device"):
        report["serial"]["open_error"] = (
            f"{args.port} does not exist or is not a character device"
        )
        report["serial"]["open_errno"] = errno.ENOENT
        verdict = classify_report(report)
        report["verdict"] = verdict
        report["finished_at_utc"] = now_iso()
        return report, int(verdict["exit_code"])

    try:
        import serial
    except ModuleNotFoundError as exc:
        report["serial"]["open_error"] = f"pyserial unavailable: {exc}"
        verdict = classify_report(report)
        report["verdict"] = verdict
        report["finished_at_utc"] = now_iso()
        return report, int(verdict["exit_code"])

    report["runtime"]["pyserial_version"] = getattr(serial, "VERSION", "unknown")
    serial_port = None
    open_kwargs = {
        "port": args.port,
        "baudrate": args.baudrate,
        "bytesize": serial.EIGHTBITS,
        "parity": serial.PARITY_NONE,
        "stopbits": serial.STOPBITS_ONE,
        "timeout": 0.10,
        "write_timeout": 1.0,
        "xonxoff": False,
        "rtscts": False,
        "dsrdtr": None,
        "exclusive": True,
    }
    report["serial"]["open_settings"] = {
        "baudrate": args.baudrate,
        "bytesize": 8,
        "parity": "N",
        "stopbits": 1,
        "xonxoff": False,
        "rtscts": False,
        "dsrdtr": None,
        "exclusive": True,
    }

    try:
        try:
            serial_port = serial.Serial(**open_kwargs)
        except TypeError as exc:
            # Old pyserial releases may not expose the POSIX-only exclusive flag.
            if "exclusive" not in str(exc):
                raise
            report["serial"]["warnings"].append(
                "This pyserial version does not support exclusive=True; retried without it."
            )
            open_kwargs.pop("exclusive")
            serial_port = serial.Serial(**open_kwargs)

        for method_name in ("setRTS", "setDTR"):
            try:
                getattr(serial_port, method_name)(False)
            except (AttributeError, OSError) as exc:
                report["serial"]["warnings"].append(
                    f"{method_name}(False) failed: {type(exc).__name__}: {exc}"
                )

        passive_raw = read_for_seconds(
            serial_port,
            args.settle_seconds,
            args.max_rx_bytes,
        )
        report["serial"]["passive_startup_rx"] = {
            "byte_count": len(passive_raw),
            "raw_utf8": passive_raw.decode("utf-8", errors="replace"),
            "raw_hex": passive_raw.hex(" "),
            "json_payloads": extract_json_objects(passive_raw),
        }

        try:
            serial_port.reset_input_buffer()
        except (AttributeError, OSError) as exc:
            report["serial"]["warnings"].append(
                f"reset_input_buffer failed: {type(exc).__name__}: {exc}"
            )

        for attempt_number in range(1, args.attempts + 1):
            attempt_started = time.monotonic()
            written = serial_port.write(STATUS_REQUEST)
            serial_port.flush()
            raw = read_for_seconds(
                serial_port,
                args.response_timeout,
                args.max_rx_bytes,
            )
            payloads = extract_json_objects(raw)
            attempt = {
                "attempt": attempt_number,
                "bytes_written": written,
                "tx_utf8": STATUS_REQUEST.decode("ascii").rstrip("\n"),
                "tx_hex": STATUS_REQUEST.hex(" "),
                "elapsed_seconds": round(time.monotonic() - attempt_started, 3),
                "byte_count": len(raw),
                "raw_utf8": raw.decode("utf-8", errors="replace"),
                "raw_hex": raw.hex(" "),
                "json_payloads": payloads,
                "matched_t1051": has_t1051(payloads),
                # Used only until classification; removed before JSON output.
                "raw_bytes_internal": raw,
            }
            report["serial"]["attempts"].append(attempt)
            if attempt["matched_t1051"]:
                break
            if attempt_number < args.attempts:
                time.sleep(args.interval_seconds)

    except (OSError, ValueError, serial.SerialException) as exc:
        report["serial"]["open_error"] = f"{type(exc).__name__}: {exc}"
        report["serial"]["open_errno"] = getattr(exc, "errno", None)
    finally:
        if serial_port is not None:
            try:
                serial_port.close()
            except OSError as exc:
                report["serial"]["warnings"].append(
                    f"close failed: {type(exc).__name__}: {exc}"
                )

    verdict = classify_report(report)
    report["verdict"] = verdict
    report["finished_at_utc"] = now_iso()
    for attempt in report["serial"]["attempts"]:
        attempt.pop("raw_bytes_internal", None)
    return report, int(verdict["exit_code"])


def write_report(report: dict[str, Any], path_text: str) -> None:
    path = Path(path_text).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(json_safe(report), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def print_summary(report: dict[str, Any], report_path: str) -> None:
    verdict = report["verdict"]
    print("\n=== RoArm UART 自动诊断 ===")
    print(verdict["summary_zh"])
    selected = report.get("selected_device", {})
    sysfs = selected.get("sysfs", {})
    attrs = sysfs.get("usb_attributes", {})
    identity = " | ".join(
        str(attrs[key])
        for key in ("manufacturer", "product", "serial", "idVendor", "idProduct")
        if attrs.get(key)
    )
    print(
        f"端口: {selected.get('path')} -> kernel "
        f"{sysfs.get('kernel_tty_name', 'unknown')}"
    )
    if identity:
        print(f"USB: {identity}")
    for attempt in report.get("serial", {}).get("attempts", []):
        print(
            f"Attempt {attempt['attempt']}: RX={attempt['byte_count']} bytes, "
            f"T=1051={attempt['matched_t1051']}"
        )
        if attempt["raw_utf8"]:
            print(f"  RX text: {attempt['raw_utf8']!r}")
    if verdict.get("likely_causes_zh"):
        print("可能原因:")
        for item in verdict["likely_causes_zh"]:
            print(f"  - {item}")
    print("下一步:")
    for item in verdict.get("next_steps_zh", []):
        print(f"  - {item}")
    print(f"详细报告: {report_path}")
    print("安全状态: 仅发送只读 T=105；未发送任何运动命令。")


def run_self_test() -> int:
    samples = [
        (
            b'boot text\r\n{"T":1051,"x":1}\r\n',
            [1051],
        ),
        (
            b'{"T":105}\nnoise\n{"T":"1051","v":1211}',
            [105, 1051],
        ),
        (b"no json here", []),
    ]
    for raw, expected_types in samples:
        actual = [payload_type(item) for item in extract_json_objects(raw)]
        if actual != expected_types:
            print(f"SELF-TEST FAIL: expected {expected_types}, got {actual}")
            return 1
    if STATUS_REQUEST != b'{"T":105}\n':
        print(f"SELF-TEST FAIL: unsafe/unexpected frame {STATUS_REQUEST!r}")
        return 1
    print("SELF-TEST PASS: parser and fixed read-only T=105 frame are correct.")
    return 0


def positive_float(value: str) -> float:
    number = float(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return number


def bounded_attempts(value: str) -> int:
    number = int(value)
    if not 1 <= number <= 10:
        raise argparse.ArgumentTypeError("must be between 1 and 10")
    return number


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only RoArm-M2-S UART diagnostic. It only sends {\"T\":105}."
        )
    )
    parser.add_argument("--port", default="/dev/ttyROARM")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--attempts", type=bounded_attempts, default=3)
    parser.add_argument("--settle-seconds", type=positive_float, default=2.0)
    parser.add_argument("--response-timeout", type=positive_float, default=5.0)
    parser.add_argument("--interval-seconds", type=positive_float, default=0.25)
    parser.add_argument("--max-rx-bytes", type=int, default=16384)
    parser.add_argument("--report", default=DEFAULT_REPORT)
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="Validate this script without opening any serial device.",
    )
    args = parser.parse_args()
    if args.baudrate <= 0:
        parser.error("--baudrate must be positive")
    if args.max_rx_bytes < 256:
        parser.error("--max-rx-bytes must be at least 256")
    return args


def main() -> int:
    args = parse_args()
    if args.self_test:
        return run_self_test()
    report, exit_code = run_diagnostic(args)
    try:
        write_report(report, args.report)
    except OSError as exc:
        print(f"Unable to write report {args.report}: {exc}", file=sys.stderr)
        print_summary(report, "<report write failed>")
        return 7 if exit_code == 0 else exit_code
    print_summary(report, args.report)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
