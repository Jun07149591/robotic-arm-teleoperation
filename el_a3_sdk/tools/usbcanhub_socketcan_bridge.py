#!/usr/bin/env python3
import argparse
import socket
import struct
import threading
import time
from typing import Optional

import serial


CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF

# Linux struct can_frame:
# can_id: uint32
# can_dlc: uint8
# padding: 3 bytes
# data: 8 bytes
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)


def pack_socketcan_frame(can_id: int, data: bytes, extended: bool = True) -> bytes:
    can_id &= CAN_EFF_MASK
    can_id_flags = can_id | (CAN_EFF_FLAG if extended else 0)
    data = data[:8]
    dlc = len(data)
    data = data.ljust(8, b"\x00")
    return struct.pack(CAN_FRAME_FMT, can_id_flags, dlc, data)


def unpack_socketcan_frame(frame: bytes):
    can_id_flags, dlc, data = struct.unpack(CAN_FRAME_FMT, frame)
    is_extended = bool(can_id_flags & CAN_EFF_FLAG)
    can_id = can_id_flags & CAN_EFF_MASK
    return can_id, data[:dlc], is_extended


def encode_usbcanhub_id(can_id: int) -> bytes:
    """
    根据官方文档示例：
    真实 CAN ID = 扩展帧字段 >> 3

    文档示例：
      扩展帧字段: 90 07 e8 0c
      右移 3 位后: 12 00 fd 01

    因此发送时：
      扩展帧字段 ≈ (真实 CAN ID << 3) | 0x04

    这里的 0x04 来自文档示例中最低 3 bit 的标志位。
    如果后续发现收发异常，需要根据厂家协议微调这个标志位。
    """
    raw = ((can_id & CAN_EFF_MASK) << 3) | 0x04
    return raw.to_bytes(4, byteorder="big")


def decode_usbcanhub_id(raw4: bytes) -> int:
    raw = int.from_bytes(raw4, byteorder="big")
    return (raw >> 3) & CAN_EFF_MASK


def make_usbcanhub_frame(can_id: int, data: bytes) -> bytes:
    """
    USB_CANHUB 串口帧格式：
      41 54 + 扩展帧4字节 + DLC1字节 + DATA + 0d 0a
    """
    data = data[:8]
    return b"AT" + encode_usbcanhub_id(can_id) + bytes([len(data)]) + data + b"\r\n"


def parse_one_usbcanhub_frame(buffer: bytearray) -> Optional[tuple[int, bytes]]:
    """
    从串口缓存中解析一帧：
      AT + 4字节ID字段 + 1字节DLC + DATA + CRLF
    """
    # 找帧头
    start = buffer.find(b"AT")
    if start < 0:
        buffer.clear()
        return None

    if start > 0:
        del buffer[:start]

    # 至少 AT + id4 + dlc1 + crlf2 = 9 字节
    if len(buffer) < 9:
        return None

    dlc = buffer[6]
    if dlc > 8:
        # DLC 不合法，丢弃帧头继续找
        del buffer[:2]
        return None

    total_len = 2 + 4 + 1 + dlc + 2
    if len(buffer) < total_len:
        return None

    frame = bytes(buffer[:total_len])
    del buffer[:total_len]

    if frame[-2:] != b"\r\n":
        return None

    can_id = decode_usbcanhub_id(frame[2:6])
    data = frame[7:7 + dlc]
    return can_id, data


class UsbCanHubBridge:
    def __init__(self, iface: str, serial_port: str, serial_baudrate: int):
        self.iface = iface
        self.serial_port = serial_port
        self.serial_baudrate = serial_baudrate

        self.running = False
        self.ser = None
        self.sock = None

        self.tx_count = 0
        self.rx_count = 0

    def open(self):
        self.ser = serial.Serial(
            self.serial_port,
            self.serial_baudrate,
            timeout=0.01,
            write_timeout=0.1,
        )

        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((self.iface,))
        self.sock.settimeout(0.1)

        self.running = True
        print(f"[OK] serial opened: {self.serial_port} @ {self.serial_baudrate}")
        print(f"[OK] socketcan opened: {self.iface}")

    def close(self):
        self.running = False
        time.sleep(0.1)

        if self.sock:
            self.sock.close()
        if self.ser:
            self.ser.close()

        print("[OK] bridge closed")

    def socketcan_to_serial_loop(self):
        while self.running:
            try:
                frame = self.sock.recv(CAN_FRAME_SIZE)
                can_id, data, is_extended = unpack_socketcan_frame(frame)

                # Robstride 一般使用扩展帧；标准帧这里也直接转
                serial_frame = make_usbcanhub_frame(can_id, data)
                self.ser.write(serial_frame)
                self.ser.flush()

                self.tx_count += 1
                print(
                    f"[TX] can0 -> serial | id=0x{can_id:08X} "
                    f"dlc={len(data)} data={data.hex(' ')} "
                    f"raw={serial_frame.hex(' ')}"
                )

            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[ERR] socketcan_to_serial: {e}")
                time.sleep(0.05)

    def serial_to_socketcan_loop(self):
        buffer = bytearray()

        while self.running:
            try:
                chunk = self.ser.read(256)
                if chunk:
                    buffer.extend(chunk)

                while True:
                    parsed = parse_one_usbcanhub_frame(buffer)
                    if parsed is None:
                        break

                    can_id, data = parsed
                    socket_frame = pack_socketcan_frame(can_id, data, extended=True)
                    self.sock.send(socket_frame)

                    self.rx_count += 1
                    print(
                        f"[RX] serial -> can0 | id=0x{can_id:08X} "
                        f"dlc={len(data)} data={data.hex(' ')}"
                    )

            except Exception as e:
                if self.running:
                    print(f"[ERR] serial_to_socketcan: {e}")
                time.sleep(0.05)

    def stats_loop(self):
        while self.running:
            time.sleep(2.0)
            print(f"[STAT] TX={self.tx_count}, RX={self.rx_count}")

    def run(self):
        self.open()

        threads = [
            threading.Thread(target=self.socketcan_to_serial_loop, daemon=True),
            threading.Thread(target=self.serial_to_socketcan_loop, daemon=True),
            threading.Thread(target=self.stats_loop, daemon=True),
        ]

        for t in threads:
            t.start()

        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\n[INFO] Ctrl+C received")
        finally:
            self.close()


def main():
    parser = argparse.ArgumentParser(
        description="USB_CANHUB AT协议 <-> SocketCAN can0 中转站"
    )
    parser.add_argument("--iface", default="can0", help="SocketCAN 接口名，默认 can0")
    parser.add_argument("--serial", default="/dev/ttyUSB0", help="USB_CANHUB 串口")
    parser.add_argument("--baudrate", type=int, default=2000000, help="串口波特率")
    args = parser.parse_args()

    bridge = UsbCanHubBridge(
        iface=args.iface,
        serial_port=args.serial,
        serial_baudrate=args.baudrate,
    )
    bridge.run()


if __name__ == "__main__":
    main()
