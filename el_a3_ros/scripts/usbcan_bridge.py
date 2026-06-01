#!/usr/bin/env python3
"""
USB_CAN 私有协议 ↔ SocketCAN (vcan0) 双向桥接

USB_CAN 模块串口帧格式:
  FRAME: 41 54 | ID[4] | DLC[1] | DATA[0..8] | 0D 0A
  其中 ID[4] 是真实 29 位 CAN ID 右移 3 位后的大端序 4 字节

用法:
  sudo python3 usbcan_bridge.py --serial /dev/ttyUSB0 --can vcan0 --bitrate 1000000
"""

import argparse
import struct
import socket
import time
import threading
import sys
import os
import ctypes
import ctypes.util

# ─── SocketCAN 常量 ───────────────────────────────────────────
PF_CAN = 29
AF_CAN = 29
SOL_CAN_RAW = 101
CAN_RAW = 1
CAN_RAW_FILTER = 26
CAN_RAW_ERR_FILTER = 27
CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF

SIOCGIFINDEX = 0x8933

class sockaddr_can(ctypes.Structure):
    _fields_ = [
        ("can_family", ctypes.c_uint16),
        ("can_ifindex", ctypes.c_int),
        ("rx_id", ctypes.c_uint32),
        ("tx_id", ctypes.c_uint32),
    ]

class ifreq(ctypes.Structure):
    _fields_ = [("ifr_name", ctypes.c_char * 16),
                ("ifr_ifindex", ctypes.c_int)]

class can_frame(ctypes.Structure):
    _fields_ = [
        ("can_id", ctypes.c_uint32),
        ("can_dlc", ctypes.c_uint8),
        ("__pad", ctypes.c_uint8),
        ("__res0", ctypes.c_uint8),
        ("__res1", ctypes.c_uint8),
        ("data", ctypes.c_uint8 * 8),
    ]


class UsbCanBridge:
    """USB_CAN 私有协议 ↔ SocketCAN 桥接器"""

    FRAME_HEADER = bytes([0x41, 0x54])  # "AT"
    FRAME_TAIL = b'\x0d\x0a'

    def __init__(self, serial_port: str, can_if: str, serial_baudrate: int = 2000000):
        self.serial_port = serial_port
        self.can_if = can_if
        self.serial_baudrate = serial_baudrate
        self._serial_fd = None
        self._can_sock = None
        self._running = False
        self._stats = {"tx": 0, "rx": 0, "tx_err": 0, "rx_err": 0}

    # ── 串口操作 ──────────────────────────────────────────────

    def _serial_open(self) -> bool:
        """打开串口并配置"""
        try:
            import serial
            self._ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.serial_baudrate,
                timeout=0.005,
                write_timeout=0.005,
            )
            self._serial_fd = self._ser.fileno()
            print(f"[USB_CAN] 串口 {self.serial_port} 已打开 @ {self.serial_baudrate} bps")
            return True
        except ImportError:
            print("[USB_CAN] pyserial 未安装，安装中...")
            os.system(f"{sys.executable} -m pip install pyserial -q")
            import serial
            self._ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.serial_baudrate,
                timeout=0.005,
                write_timeout=0.005,
            )
            self._serial_fd = self._ser.fileno()
            print(f"[USB_CAN] 串口 {self.serial_port} 已打开 @ {self.serial_baudrate} bps")
            return True
        except Exception as e:
            print(f"[USB_CAN] 串口打开失败: {e}")
            return False

    def _serial_read_into(self, buf: bytearray):
        """从串口读取数据到缓冲区"""
        try:
            data = self._ser.read(max(1, self._ser.in_waiting))
            if data:
                # 只要有数据就打印（查看原始格式）
                print(f"[RAW] 串口← {len(data)}字节: {data.hex(' ')} | ascii: {repr(data[:min(40,len(data))])}")
                buf.extend(data)
        except Exception:
            pass

    def _serial_write(self, data: bytes):
        """写入串口"""
        try:
            self._ser.write(data)
            return True
        except Exception as e:
            self._stats["tx_err"] += 1
            if self._stats["tx_err"] % 50 == 0:
                print(f"[USB_CAN] 串口写入错误: {e}")
            return False

    # ── SocketCAN 操作 ─────────────────────────────────────────

    def _can_open(self) -> bool:
        """打开 vcan 接口"""
        try:
            self._can_sock = socket.socket(PF_CAN, socket.SOCK_RAW, CAN_RAW)
            ifr = ifreq()
            ifr.ifr_name = self.can_if.encode()[:15]

            libc = ctypes.CDLL(ctypes.util.find_library("c"), use_errno=True)
            if libc.ioctl(self._can_sock.fileno(), SIOCGIFINDEX, ctypes.byref(ifr)) < 0:
                err = ctypes.get_errno()
                print(f"[USB_CAN] 获取 {self.can_if} 接口索引失败: {os.strerror(err)}")
                print(f"[USB_CAN] 请先执行: sudo ip link add dev {self.can_if} type vcan && sudo ip link set {self.can_if} up")
                self._can_sock.close()
                self._can_sock = None
                return False

            addr = sockaddr_can()
            addr.can_family = AF_CAN
            addr.can_ifindex = ifr.ifr_ifindex

            libc.bind(self._can_sock.fileno(), ctypes.byref(addr), ctypes.sizeof(addr))

            print(f"[USB_CAN] vcan 接口 {self.can_if} 已绑定")
            return True
        except Exception as e:
            print(f"[USB_CAN] SocketCAN 初始化失败: {e}")
            if self._can_sock:
                self._can_sock.close()
                self._can_sock = None
            return False

    def _can_write(self, can_id: int, data: bytes):
        """写入 CAN 帧到 vcan"""
        if not self._can_sock:
            return
        frame = can_frame()
        frame.can_id = can_id | CAN_EFF_FLAG
        frame.can_dlc = min(len(data), 8)
        for i in range(frame.can_dlc):
            frame.data[i] = data[i]

        libc = ctypes.CDLL(ctypes.util.find_library("c"), use_errno=True)
        n = libc.write(self._can_sock.fileno(), ctypes.byref(frame), ctypes.sizeof(frame))
        if n < 0:
            self._stats["tx_err"] += 1

    def _can_read(self) -> bool:
        """从 vcan 读取 CAN 帧并发送到串口"""
        if not self._can_sock:
            return False
        frame = can_frame()
        libc = ctypes.CDLL(ctypes.util.find_library("c"), use_errno=True)
        n = libc.read(self._can_sock.fileno(), ctypes.byref(frame), ctypes.sizeof(frame))
        if n <= 0:
            return False

        can_id = frame.can_id & CAN_EFF_MASK
        dlc = frame.can_dlc
        data = bytes(frame.data[:dlc])

        # 构造 USB_CAN 私有协议帧
        serial_id = can_id << 3  # 规格书：CAN ID 左移 3 位存储
        id_bytes = struct.pack(">I", serial_id)

        packet = b'\x41\x54' + id_bytes + bytes([dlc]) + data + b'\x0d\x0a'

        # 前几帧打印调试（验证格式）
        if self._stats["tx"] < 3:
            print(f"[DBG] CAN→串口: can_id=0x{can_id:08X} → serial_id=0x{int.from_bytes(id_bytes,'big'):08X} "
                  f"packet={packet.hex(' ')}")

        self._serial_write(packet)
        self._stats["tx"] += 1
        return True

    # ── USB_CAN 协议解析 ──────────────────────────────────────

    def _parse_usbcan_frame(self, raw: bytes):
        """解析 USB_CAN 私有协议帧 → 发送到 vcan"""
        if len(raw) < 7:
            return False

        # 查找帧头 41 54
        header_idx = raw.find(self.FRAME_HEADER)
        if header_idx < 0:
            return False

        tail_idx = raw.find(self.FRAME_TAIL, header_idx + 6)
        if tail_idx < 0:
            return False

        frame_body = raw[header_idx + 2:tail_idx]  # 去掉 AT 和 \r\n
        if len(frame_body) < 5:
            return True  # 已消费但不完整

        shifted_id = struct.unpack(">I", frame_body[0:4])[0]
        can_id = shifted_id >> 3  # 规格书：右移 3 位恢复真实 CAN ID
        dlc = frame_body[4]
        data = frame_body[5:5 + min(dlc, 8)]

        # 写入 vcan
        self._can_write(can_id, data)
        self._stats["rx"] += 1

        return True

    # ── 主循环 ─────────────────────────────────────────────────

    def run(self):
        """启动双向桥接"""
        if not self._serial_open():
            return
        if not self._can_open():
            self._ser.close()
            return

        self._running = True
        ser_buf = bytearray()
        last_stat = time.time()

        print(f"[USB_CAN] 桥接已启动: {self.serial_port} ↔ {self.can_if}")
        print("[USB_CAN] 按 Ctrl+C 停止")

        try:
            while self._running:
                # 串口 → vcan
                self._serial_read_into(ser_buf)
                while self._parse_usbcan_frame(ser_buf):
                    # 清理已解析的数据
                    idx = ser_buf.find(self.FRAME_TAIL)
                    if idx >= 0:
                        ser_buf = ser_buf[idx + 2:]
                    else:
                        break

                # vcan → 串口（不阻塞）
                self._can_read()

                # 定期统计
                if time.time() - last_stat > 5.0:
                    print(f"[STAT] SER→CAN(串→CAN) RX={self._stats['rx']}, "
                          f"CAN→SER(CAN→串) TX={self._stats['tx']}")
                    last_stat = time.time()

        except KeyboardInterrupt:
            pass
        finally:
            self._running = False
            if self._ser and self._ser.is_open:
                self._ser.close()
            if self._can_sock:
                self._can_sock.close()
            print(f"\n[USB_CAN] 桥接已停止. 最终统计: RX={self._stats['rx']}, TX={self._stats['tx']}")


def main():
    parser = argparse.ArgumentParser(description="USB_CAN ↔ SocketCAN 双向桥接")
    parser.add_argument("--serial", default="/dev/ttyUSB0", help="USB_CAN 串口设备")
    parser.add_argument("--can", default="vcan0", help="SocketCAN 接口名")
    parser.add_argument("--baudrate", type=int, default=2000000, help="串口波特率")
    args = parser.parse_args()

    bridge = UsbCanBridge(args.serial, args.can, args.baudrate)
    bridge.run()


if __name__ == "__main__":
    main()
