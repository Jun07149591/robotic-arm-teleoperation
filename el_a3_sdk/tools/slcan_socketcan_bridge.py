#!/usr/bin/env python3
import argparse
import socket
import struct
import threading
import time

import serial


CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF

CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

SLCAN_BITRATE_MAP = {
    10000: "S0",
    20000: "S1",
    50000: "S2",
    100000: "S3",
    125000: "S4",
    250000: "S5",
    500000: "S6",
    800000: "S7",
    1000000: "S8",
}


def unpack_socketcan_frame(frame: bytes):
    can_id_flags, dlc, data = struct.unpack(CAN_FRAME_FMT, frame)
    is_extended = bool(can_id_flags & CAN_EFF_FLAG)
    can_id = can_id_flags & CAN_EFF_MASK
    return can_id, data[:dlc], is_extended


def pack_socketcan_frame(can_id: int, data: bytes, extended: bool = True):
    can_id_flags = can_id & CAN_EFF_MASK
    if extended:
        can_id_flags |= CAN_EFF_FLAG
    data = data[:8]
    dlc = len(data)
    return struct.pack(CAN_FRAME_FMT, can_id_flags, dlc, data.ljust(8, b"\x00"))


def make_slcan_frame(can_id: int, data: bytes, extended: bool = True) -> bytes:
    data = data[:8]
    if extended:
        return f"T{can_id & CAN_EFF_MASK:08X}{len(data):X}{data.hex().upper()}\r".encode("ascii")
    else:
        return f"t{can_id & 0x7FF:03X}{len(data):X}{data.hex().upper()}\r".encode("ascii")


def parse_slcan_line(line: bytes):
    if not line:
        return None

    frame_type = line[:1]

    try:
        if frame_type == b"T":
            if len(line) < 10:
                return None
            can_id = int(line[1:9], 16)
            dlc = int(line[9:10], 16)
            data = bytes.fromhex(line[10:10 + dlc * 2].decode("ascii"))
            return can_id, data, True

        if frame_type == b"t":
            if len(line) < 5:
                return None
            can_id = int(line[1:4], 16)
            dlc = int(line[4:5], 16)
            data = bytes.fromhex(line[5:5 + dlc * 2].decode("ascii"))
            return can_id, data, False

    except Exception:
        return None

    return None


class SlcanSocketcanBridge:
    def __init__(self, iface, serial_port, serial_baudrate, can_bitrate):
        self.iface = iface
        self.serial_port = serial_port
        self.serial_baudrate = serial_baudrate
        self.can_bitrate = can_bitrate

        self.ser = None
        self.sock = None
        self.running = False
        self.tx_count = 0
        self.rx_count = 0

    def slcan_cmd(self, cmd: str):
        raw = (cmd + "\r").encode("ascii")
        self.ser.write(raw)
        self.ser.flush()
        print(f"[CMD] {cmd}")

    def open(self):
        self.ser = serial.Serial(
            self.serial_port,
            self.serial_baudrate,
            timeout=0.01,
            write_timeout=0.1,
        )
        time.sleep(0.05)

        # 按 SDK 的 slcan_can_driver.py 初始化流程：C -> Sx -> O
        self.slcan_cmd("C")
        time.sleep(0.02)

        bitrate_cmd = SLCAN_BITRATE_MAP.get(self.can_bitrate)
        if bitrate_cmd is None:
            raise RuntimeError(f"不支持的 CAN 波特率: {self.can_bitrate}")

        self.slcan_cmd(bitrate_cmd)
        time.sleep(0.02)

        self.slcan_cmd("O")
        time.sleep(0.02)

        self.ser.reset_input_buffer()

        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((self.iface,))
        self.sock.settimeout(0.1)

        self.running = True
        print(f"[OK] serial opened: {self.serial_port} @ {self.serial_baudrate}")
        print(f"[OK] can bitrate: {self.can_bitrate}")
        print(f"[OK] socketcan opened: {self.iface}")

    def close(self):
        self.running = False
        time.sleep(0.1)

        try:
            if self.ser and self.ser.is_open:
                self.slcan_cmd("C")
        except Exception:
            pass

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

                raw = make_slcan_frame(can_id, data, extended=True)
                self.ser.write(raw)
                self.ser.flush()

                self.tx_count += 1
                print(
                    f"[TX] can0 -> serial | id=0x{can_id:08X} "
                    f"dlc={len(data)} data={data.hex(' ')} raw={raw!r}"
                )

            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[ERR] TX loop: {e}")
                time.sleep(0.05)

    def serial_to_socketcan_loop(self):
        buf = b""

        while self.running:
            try:
                chunk = self.ser.read(256)
                if chunk:
                    buf += chunk

                while b"\r" in buf:
                    line, buf = buf.split(b"\r", 1)
                    if not line:
                        continue

                    parsed = parse_slcan_line(line)
                    if parsed is None:
                        print(f"[RAW] {line!r}")
                        continue

                    can_id, data, is_extended = parsed
                    frame = pack_socketcan_frame(can_id, data, extended=is_extended)
                    self.sock.send(frame)

                    self.rx_count += 1
                    print(
                        f"[RX] serial -> can0 | id=0x{can_id:08X} "
                        f"dlc={len(data)} data={data.hex(' ')}"
                    )

            except Exception as e:
                if self.running:
                    print(f"[ERR] RX loop: {e}")
                time.sleep(0.05)

    def stats_loop(self):
        while self.running:
            time.sleep(2.0)
            print(f"[STAT] TX={self.tx_count}, RX={self.rx_count}")

    def run(self):
        self.open()

        threading.Thread(target=self.socketcan_to_serial_loop, daemon=True).start()
        threading.Thread(target=self.serial_to_socketcan_loop, daemon=True).start()
        threading.Thread(target=self.stats_loop, daemon=True).start()

        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\n[INFO] Ctrl+C")
        finally:
            self.close()


def main():
    parser = argparse.ArgumentParser(description="Lawicel SLCAN <-> SocketCAN bridge")
    parser.add_argument("--iface", default="can0")
    parser.add_argument("--serial", default="/dev/ttyUSB0")
    parser.add_argument("--serial-baudrate", type=int, default=2000000)
    parser.add_argument("--can-bitrate", type=int, default=1000000)
    args = parser.parse_args()

    bridge = SlcanSocketcanBridge(
        iface=args.iface,
        serial_port=args.serial,
        serial_baudrate=args.serial_baudrate,
        can_bitrate=args.can_bitrate,
    )
    bridge.run()


if __name__ == "__main__":
    main()
