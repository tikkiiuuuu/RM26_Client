#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# SPDX-License-Identifier: MIT
"""
自定义控制器链路联调工具（RS232-USB 串口 / UDDP 兼容）

@file controller_link_emulator.py
@brief 在没有 2027_controller 硬件时，按自定义控制器协议生成 0x0302 帧，
       经虚拟串口（推荐）或 UDP 送到客户端 CustomControllerReceiver，
       用于验证合并后的 RS232-USB 链路与解析实现。

帧格式与 2027_controller 的发送端字节级一致：
    SOF(0xA5) | data_length(2, 小端) | seq(1) | CRC8(1)
    | cmd_id(2, 小端, 0x0302) | data(30) | CRC16(2, 小端)

控制段布局（30 字节）：
    [0..3]   roll        float32 小端
    [4..7]   encoder_x   float32 小端
    [8..11]  encoder_y   float32 小端
    [12..15] pitch1      float32 小端
    [16..19] pitch2      float32 小端
    [20..23] yaw         float32 小端
    [24]     mode        0=无力 1=底盘常规 2=机械臂自定义
    [25]     slow_open
    [26]     slow_close
    [27]     按键位
    [28..29] 保留

典型用法：

  1) 串口路径（推荐，最贴近现场链路）
     # 终端 A：建一对虚拟串口，客户端连 /dev/pts/N，工具写另一端
     socat -d -d pty,raw,echo=0,link=/tmp/rm26-controller-client \\
                pty,raw,echo=0,link=/tmp/rm26-controller-emulator
     # 终端 B：客户端使用 RM_CONTROLLER_SERIAL=/tmp/rm26-controller-client
     python3 tools/controller_link_emulator.py --serial /tmp/rm26-controller-emulator

  2) UDP 兼容路径（客户端 serial=false）
     python3 tools/controller_link_emulator.py --udp-port 3340

  3) 仅自检 CRC 与组帧，不发数据
     python3 tools/controller_link_emulator.py --selftest
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
import time

SOF = 0xA5
CMD_ID_CUSTOM_CONTROLLER = 0x0302
CONTROL_DATA_BYTES = 30
DEFAULT_BAUD = 115200
DEFAULT_UDP_PORT = 3340
# 控制器 Video_Link_Task 的发送周期（TX_PERIOD = 34 ms，约 29 Hz）。
DEFAULT_PERIOD_MS = 34
# 单元测试使用的黄金帧（seq=7）；用于自检本工具的 CRC 与组帧实现。
GOLDEN_FRAME_SEQ7 = bytes.fromhex(
    "a51e0007fe0203"
    "0000c03f0000803e000000bf"
    "0000803f000000400000a0bf"
    "020100060000290c"
)


def build_crc8_table() -> list[int]:
    """协议附录 CRC8：poly 0x31 的反射形式 0x8C，逐位 LSB-first。"""
    table = []
    for i in range(256):
        value = i
        for _ in range(8):
            value = ((value >> 1) ^ 0x8C) if (value & 1) else (value >> 1)
        table.append(value & 0xFF)
    return table


def build_crc16_table() -> list[int]:
    """协议附录 CRC16：poly 0x1021 的反射形式 0x8408，逐位 LSB-first。"""
    table = []
    for i in range(256):
        value = i
        for _ in range(8):
            value = ((value >> 1) ^ 0x8408) if (value & 1) else (value >> 1)
        table.append(value & 0xFFFF)
    return table


CRC8_TAB = build_crc8_table()
CRC16_TAB = build_crc16_table()


def crc8(data: bytes, init: int = 0xFF) -> int:
    value = init
    for byte in data:
        value = CRC8_TAB[value ^ byte]
    return value


def crc16(data: bytes, init: int = 0xFFFF) -> int:
    value = init
    for byte in data:
        value = (value >> 8) ^ CRC16_TAB[(value ^ byte) & 0xFF]
    return value & 0xFFFF


def build_frame(cmd_id: int, seq: int, payload: bytes) -> bytes:
    header = bytes([SOF]) + struct.pack("<H", len(payload)) + bytes([seq & 0xFF])
    header += bytes([crc8(header)])
    body = header + struct.pack("<H", cmd_id) + payload
    return body + struct.pack("<H", crc16(body))


def build_control_payload(roll: float, enc_x: float, enc_y: float, pitch1: float,
                          pitch2: float, yaw: float, mode: int, slow_open: int,
                          slow_close: int, buttons: int) -> bytes:
    payload = struct.pack("<ffffff", roll, enc_x, enc_y, pitch1, pitch2, yaw)
    payload += bytes([mode & 0xFF, slow_open & 0x01, slow_close & 0x01,
                      buttons & 0xFF, 0, 0])
    assert len(payload) == CONTROL_DATA_BYTES, len(payload)
    return payload


def sin_payload(phase: float) -> bytes:
    """按 2027_controller 的 USE_SIN_SIGNAL 调试模式生成一帧控制段。

    控制器在该模式下机械臂通信停用，6 路 float 由 sin 函数填充（同频不同相），
    mode/slow_open/slow_close 固定为 0，便于上位机直接观察解析结果。
    """
    amplitude = 1.0
    channel_phase_step = math.pi / 6.0
    values = [
        amplitude * math.sin(phase + channel * channel_phase_step)
        for channel in range(6)
    ]
    payload = struct.pack("<ffffff", *values)
    payload += bytes([0, 0, 0, 0, 0, 0])
    assert len(payload) == CONTROL_DATA_BYTES, len(payload)
    return payload


def selftest() -> int:
    """用单元测试同源黄金帧验证本工具的组帧与 CRC 实现。"""
    payload = build_control_payload(1.5, 0.25, -0.5, 1.0, 2.0, -1.25, 2, 1, 0,
                                    0b00000110)
    frame = build_frame(CMD_ID_CUSTOM_CONTROLLER, 7, payload)
    if frame != GOLDEN_FRAME_SEQ7:
        print("[selftest] 组帧结果与黄金帧不一致")
        print("  期望:", GOLDEN_FRAME_SEQ7.hex())
        print("  实际:", frame.hex())
        return 1
    print("[selftest] 与 C++ 单元测试黄金帧一致，CRC/组帧实现正确")
    return 0


def open_serial(path: str, baud: int):
    try:
        import serial  # type: ignore
    except ImportError:
        print("缺少 pyserial：pip install pyserial", file=sys.stderr)
        return None
    try:
        return serial.Serial(path, baudrate=baud, timeout=0.1, write_timeout=0.5)
    except Exception as error:  # noqa: BLE001 - 面向操作的诊断
        print(f"打开串口 {path} 失败：{error}", file=sys.stderr)
        return None


def open_udp(host: str, port: int):
    try:
        return socket.socket(socket.AF_INET, socket.SOCK_DGRAM), (host, port)
    except OSError as error:
        print(f"创建 UDP socket 失败：{error}", file=sys.stderr)
        return None


def run_loop(send, period_s: float, duration_s: float, seq_start: int,
             use_sin: bool) -> int:
    """按周期发送 0x0302 帧；duration_s <= 0 表示持续到手动中断。"""
    seq = seq_start & 0xFF
    phase = 0.0
    two_pi = 2.0 * math.pi
    freq_hz = 0.5
    sent = 0
    start = time.monotonic()
    try:
        while True:
            if duration_s > 0 and (time.monotonic() - start) >= duration_s:
                break
            if use_sin:
                payload = sin_payload(phase)
                phase += two_pi * freq_hz * period_s
                if phase >= two_pi:
                    phase -= two_pi
            else:
                payload = build_control_payload(
                    roll=0.5 * math.sin(phase),
                    enc_x=0.0,
                    enc_y=0.0,
                    pitch1=1.0,
                    pitch2=2.0,
                    yaw=-1.25,
                    mode=2,
                    slow_open=1,
                    slow_close=0,
                    buttons=0x06,
                )
                phase += two_pi * freq_hz * period_s
                if phase >= two_pi:
                    phase -= two_pi
            frame = build_frame(CMD_ID_CUSTOM_CONTROLLER, seq, payload)
            send(frame)
            seq = (seq + 1) & 0xFF
            sent += 1
            time.sleep(period_s)
    except KeyboardInterrupt:
        print("\n收到中断，停止发送")
    print(f"[emulator] 已发送 {sent} 帧 (seq 回绕 {sent // 256} 次)")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="自定义控制器 0x0302 帧联调工具（RS232-USB 串口 / UDP）")
    parser.add_argument("--serial", metavar="PATH",
                        help="写入虚拟串口，如 /tmp/rm26-controller-emulator")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help=f"串口波特率（默认 {DEFAULT_BAUD}）")
    parser.add_argument("--udp-port", type=int, default=DEFAULT_UDP_PORT,
                        help=f"UDP 目标端口（默认 {DEFAULT_UDP_PORT}）")
    parser.add_argument("--udp-host", default="127.0.0.1",
                        help="UDP 目标地址（默认 127.0.0.1）")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="发送时长（秒）；0 表示持续到 Ctrl+C")
    parser.add_argument("--period-ms", type=float, default=DEFAULT_PERIOD_MS,
                        help=f"发送周期毫秒（默认 {DEFAULT_PERIOD_MS}）")
    parser.add_argument("--seq", type=int, default=0, help="起始帧序号")
    parser.add_argument("--sin", action="store_true",
                        help="模拟 USE_SIN_SIGNAL 调试模式（6 路 sin 数据）")
    parser.add_argument("--selftest", action="store_true",
                        help="仅校验 CRC 与组帧，不发送数据")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.selftest:
        return selftest()

    period_s = max(args.period_ms, 1.0) / 1000.0

    if args.serial:
        serial_port = open_serial(args.serial, args.baud)
        if serial_port is None:
            return 2

        def send(frame: bytes) -> None:
            serial_port.write(frame)
            serial_port.flush()

        print(f"[emulator] 串口 {args.serial} @ {args.baud} 8N1，"
              f"周期 {args.period_ms:.1f} ms")
        return run_loop(send, period_s, args.duration, args.seq, args.sin)

    udp = open_udp(args.udp_host, args.udp_port)
    if udp is None:
        return 2
    sock, target = udp

    def send(frame: bytes) -> None:
        sock.sendto(frame, target)

    print(f"[emulator] UDP {args.udp_host}:{args.udp_port}，"
          f"周期 {args.period_ms:.1f} ms（需客户端 controller.serial=false）")
    return run_loop(send, period_s, args.duration, args.seq, args.sin)


if __name__ == "__main__":
    raise SystemExit(main())