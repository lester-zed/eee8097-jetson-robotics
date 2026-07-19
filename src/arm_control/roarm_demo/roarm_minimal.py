#!/usr/bin/env python3

import argparse
import json
import sys
import time

import serial
from serial import SerialException


BAUD_RATE = 115200


def read_responses(ser: serial.Serial, duration: float = 1.0) -> None:
    """在指定时间内读取并打印机械臂返回的信息。"""
    deadline = time.monotonic() + duration

    while time.monotonic() < deadline:
        raw = ser.readline()

        if raw:
            text = raw.decode("utf-8", errors="replace").strip()
            if text:
                print(f"RX: {text}")


def send_command(
    ser: serial.Serial,
    command: dict,
    wait_seconds: float = 1.0,
) -> None:
    """将 Python 字典编码成一行 JSON，通过串口发送。"""
    message = json.dumps(command, separators=(",", ":"))

    print(f"TX: {message}")

    ser.write((message + "\n").encode("utf-8"))
    ser.flush()

    read_responses(ser, wait_seconds)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Minimal USB UART controller for Waveshare RoArm-M2-S"
    )
    parser.add_argument(
        "port",
        help="Serial device, for example /dev/ttyUSB0",
    )
    args = parser.parse_args()

    try:
        with serial.Serial(
            port=args.port,
            baudrate=BAUD_RATE,
            timeout=0.2,
            write_timeout=1.0,
            dsrdtr=False,
            rtscts=False,
        ) as ser:
            # 与 Waveshare 官方示例保持一致，关闭 RTS 和 DTR。
            ser.setRTS(False)
            ser.setDTR(False)

            print(f"Connected to {args.port} at {BAUD_RATE} baud.")

            # 给 ESP32 和串口一些稳定时间。
            time.sleep(2.0)

            # 清除连接建立时可能残留的启动日志。
            ser.reset_input_buffer()

            # 1. 无机械运动测试：打开 LED。
            send_command(
                ser,
                {"T": 114, "led": 255},
                wait_seconds=0.5,
            )

            # 2. 查询末端坐标、关节角度、负载和电压。
            send_command(
                ser,
                {"T": 105},
                wait_seconds=1.5,
            )

            print()
            print("Non-motion UART test completed.")
            print("Type MOVE to perform a small base-joint movement.")
            confirmation = input("> ").strip().upper()

            if confirmation == "MOVE":
                # 3. 回到机械臂初始姿态。
                send_command(
                    ser,
                    {"T": 100},
                    wait_seconds=4.0,
                )

                # 4. 仅将底座从 0° 缓慢转到 +15°。
                send_command(
                    ser,
                    {
                        "T": 121,
                        "joint": 1,
                        "angle": 15,
                        "spd": 10,
                        "acc": 10,
                    },
                    wait_seconds=2.5,
                )

                # 5. 底座返回 0°。
                send_command(
                    ser,
                    {
                        "T": 121,
                        "joint": 1,
                        "angle": 0,
                        "spd": 10,
                        "acc": 10,
                    },
                    wait_seconds=2.5,
                )
            else:
                print("Movement skipped.")

            # 6. 关闭 LED。
            send_command(
                ser,
                {"T": 114, "led": 0},
                wait_seconds=0.5,
            )

            print("Demo finished successfully.")
            return 0

    except PermissionError:
        print(
            f"Permission denied: {args.port}\n"
            "Add your user to the dialout group and reconnect SSH:\n"
            '  sudo usermod -aG dialout "$USER"',
            file=sys.stderr,
        )
        return 1

    except SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 1

    except KeyboardInterrupt:
        print("\nStopped by user.")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
