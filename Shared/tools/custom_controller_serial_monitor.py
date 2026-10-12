#!/usr/bin/env python3
"""交互选择串口并解析自定义控制器39字节数据帧。"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from dataclasses import dataclass

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    print("缺少 pyserial，请执行：python -m pip install pyserial", file=sys.stderr)
    raise SystemExit(2) from exc


FRAME_SOF = 0xA5
FRAME_LENGTH = 39
HEADER_LENGTH = 5
PAYLOAD_LENGTH = 30
COMMAND_ID = 0x0302
CRC8_INIT = 0xFF
CRC16_INIT = 0x0000
JOINT_NAMES = (
    "big_yaw",
    "pitch1",
    "pitch2",
    "roll2",
    "pitch3",
    "roll3",
)


def crc8(data: bytes | bytearray, initial: int = CRC8_INIT) -> int:
    """DJI CRC8，反射多项式0x8C，等价于发送端CRC8_TAB。"""
    value = initial
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ 0x8C if value & 1 else value >> 1
    return value & 0xFF


def crc16(data: bytes | bytearray, initial: int = CRC16_INIT) -> int:
    """DJI CRC16，反射多项式0x8408；发送端与H7统一使用初值0x0000。"""
    value = initial
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ 0x8408 if value & 1 else value >> 1
    return value & 0xFFFF


@dataclass(frozen=True)
class ControllerFrame:
    sequence: int
    work_mode: int
    joint_target_rad: tuple[float, float, float, float, float, float]
    button_pressed: bool
    raw: bytes


class FrameParser:
    """保留跨串口read边界的数据，并在坏帧后重新搜索SOF。"""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.valid_frames = 0
        self.invalid_headers = 0
        self.invalid_frames = 0

    def feed(self, data: bytes) -> list[ControllerFrame]:
        self.buffer.extend(data)
        frames: list[ControllerFrame] = []

        while True:
            sof_index = self.buffer.find(FRAME_SOF)
            if sof_index < 0:
                self.buffer.clear()
                break
            if sof_index > 0:
                del self.buffer[:sof_index]
            if len(self.buffer) < HEADER_LENGTH:
                break

            payload_length = struct.unpack_from("<H", self.buffer, 1)[0]
            header_valid = (
                payload_length == PAYLOAD_LENGTH
                and crc8(self.buffer[: HEADER_LENGTH - 1])
                == self.buffer[HEADER_LENGTH - 1]
            )
            if not header_valid:
                self.invalid_headers += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < FRAME_LENGTH:
                break

            raw = bytes(self.buffer[:FRAME_LENGTH])
            expected_crc16 = struct.unpack_from("<H", raw, FRAME_LENGTH - 2)[0]
            if crc16(raw[:-2]) != expected_crc16:
                self.invalid_frames += 1
                del self.buffer[0]
                continue

            del self.buffer[:FRAME_LENGTH]
            command_id = struct.unpack_from("<H", raw, HEADER_LENGTH)[0]
            if command_id != COMMAND_ID:
                self.invalid_frames += 1
                continue

            work_mode, *values = struct.unpack_from("<B6fB", raw, 7)
            joint_values = tuple(float(value) for value in values[:6])
            button_value = int(values[6])
            if (
                not all(math.isfinite(value) for value in joint_values)
                or button_value not in (0, 1)
            ):
                self.invalid_frames += 1
                continue

            frames.append(
                ControllerFrame(
                    sequence=raw[3],
                    work_mode=work_mode,
                    joint_target_rad=joint_values,  # type: ignore[arg-type]
                    button_pressed=button_value == 1,
                    raw=raw,
                )
            )
            self.valid_frames += 1

        return frames


def available_ports() -> list[list_ports.ListPortInfo]:
    return sorted(list_ports.comports(), key=lambda port: port.device)


def select_port(requested_port: str | None) -> str:
    if requested_port:
        return requested_port

    ports = available_ports()
    if not ports:
        raise RuntimeError("没有检测到可用串口")

    print("可用串口：")
    for index, port in enumerate(ports, start=1):
        description = port.description or "无描述"
        print(f"  {index}. {port.device:<8} {description}")

    while True:
        choice = input(f"请选择串口 [1-{len(ports)}]：").strip()
        if choice.isdigit() and 1 <= int(choice) <= len(ports):
            return ports[int(choice) - 1].device
        print("输入无效，请输入列表中的序号。")


def format_frame(frame: ControllerFrame) -> str:
    joints = "  ".join(
        f"{name}={value:+.4f}"
        for name, value in zip(JOINT_NAMES, frame.joint_target_rad)
    )
    button = "按下" if frame.button_pressed else "松开"
    return (
        f"seq={frame.sequence:3d}  mode={frame.work_mode}  "
        f"{joints}  button={button}"
    )


def run_monitor(port: str, baudrate: int, show_raw: bool) -> None:
    parser = FrameParser()
    last_frame_time = time.monotonic()
    stale_shown = False

    print(f"打开 {port}，{baudrate} 8N1；Ctrl+C退出")
    print("通道顺序：big_yaw, pitch1, pitch2, roll2, pitch3, roll3（rad）")

    with serial.Serial(port, baudrate, timeout=0.1) as connection:
        while True:
            data = connection.read(max(1, connection.in_waiting))
            for frame in parser.feed(data):
                last_frame_time = time.monotonic()
                stale_shown = False
                print(format_frame(frame))
                if show_raw:
                    print("raw=" + frame.raw.hex(" "))

            if not stale_shown and time.monotonic() - last_frame_time > 1.0:
                print("[等待合法数据帧……]")
                stale_shown = True


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="选择串口并解析自定义控制器39字节数据帧"
    )
    parser.add_argument("--port", help="直接指定串口，例如 COM21")
    parser.add_argument("--baud", type=int, default=115200, help="波特率，默认115200")
    parser.add_argument("--raw", action="store_true", help="同时打印合法帧原始字节")
    parser.add_argument("--list", action="store_true", help="列出串口后退出")
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    if arguments.list:
        for port in available_ports():
            print(f"{port.device}\t{port.description}")
        return 0

    try:
        port = select_port(arguments.port)
        run_monitor(port, arguments.baud, arguments.raw)
    except KeyboardInterrupt:
        print("\n已停止。")
        return 0
    except (RuntimeError, serial.SerialException) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
