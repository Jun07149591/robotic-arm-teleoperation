"""CAN 接口检测与系统级操作（开启/关闭/状态查询）+ 串口检测"""

import subprocess
import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple

logger = logging.getLogger("debugger.can_utils")

ARPHRD_CAN = 280


def detect_can_interfaces() -> List[Dict]:
    """
    扫描系统中所有 CAN 类型的网络接口。
    返回 [{"name": "can0", "state": "UP"|"DOWN", "bitrate": 1000000}, ...]
    """
    interfaces = []
    net_dir = Path("/sys/class/net")
    if not net_dir.exists():
        return interfaces

    for iface_dir in sorted(net_dir.iterdir()):
        type_file = iface_dir / "type"
        if not type_file.exists():
            continue
        try:
            iface_type = int(type_file.read_text().strip())
        except (ValueError, OSError):
            continue
        if iface_type != ARPHRD_CAN:
            continue

        name = iface_dir.name
        state = get_can_state(name)
        bitrate = get_can_bitrate(name)
        interfaces.append({
            "name": name,
            "state": state,
            "bitrate": bitrate,
        })

    return interfaces


def get_can_state(iface: str) -> str:
    """读取 CAN 接口的 operstate：'UP' / 'DOWN' / 'UNKNOWN'"""
    operstate_file = Path(f"/sys/class/net/{iface}/operstate")
    try:
        raw = operstate_file.read_text().strip().upper()
        if raw == "UNKNOWN":
            flags_file = Path(f"/sys/class/net/{iface}/flags")
            if flags_file.exists():
                flags = int(flags_file.read_text().strip(), 16)
                return "UP" if flags & 0x1 else "DOWN"
        return raw
    except (FileNotFoundError, OSError):
        return "UNKNOWN"


def get_can_bitrate(iface: str) -> int:
    """读取 CAN 接口当前的波特率（bps），失败返回 0"""
    bitrate_file = Path(f"/sys/class/net/{iface}/can_bittiming/bitrate")
    try:
        return int(bitrate_file.read_text().strip())
    except (FileNotFoundError, ValueError, OSError):
        return 0


def setup_can_interface(iface: str, bitrate: int = 1000000) -> Tuple[bool, str]:
    """
    开启 CAN 接口：先 down 再配置 bitrate 再 up。
    需要 root 权限，通过 pkexec 提权。
    返回 (成功, 消息)。
    """
    commands = [
        ["ip", "link", "set", iface, "down"],
        ["ip", "link", "set", iface, "type", "can", "bitrate", str(bitrate)],
        ["ip", "link", "set", iface, "txqueuelen", "1000"],
        ["ip", "link", "set", iface, "up"],
    ]

    for cmd in commands:
        ok, msg = _run_privileged(cmd)
        if not ok:
            return False, msg

    final_state = get_can_state(iface)
    final_bitrate = get_can_bitrate(iface)
    return True, f"{iface} 已开启 ({final_bitrate} bps, {final_state})"


def shutdown_can_interface(iface: str) -> Tuple[bool, str]:
    """关闭 CAN 接口。返回 (成功, 消息)。"""
    ok, msg = _run_privileged(["ip", "link", "set", iface, "down"])
    if ok:
        return True, f"{iface} 已关闭"
    return False, msg


def _run_privileged(cmd: List[str]) -> Tuple[bool, str]:
    """
    执行需要 root 权限的命令。
    优先尝试 sudo（适用于 nopasswd 场景），
    失败后尝试 pkexec（图形化提权弹窗）。
    """
    for prefix in [["sudo", "-n"], ["pkexec"]]:
        try:
            full_cmd = prefix + cmd
            result = subprocess.run(
                full_cmd,
                capture_output=True, text=True, timeout=10,
            )
            if result.returncode == 0:
                return True, result.stdout.strip()
            if prefix[0] == "sudo":
                continue
            return False, f"命令失败: {result.stderr.strip() or result.stdout.strip()}"
        except FileNotFoundError:
            continue
        except subprocess.TimeoutExpired:
            return False, "命令执行超时"
        except Exception as e:
            return False, str(e)

    return False, "需要 root 权限：sudo 和 pkexec 均不可用"


def detect_serial_ports() -> List[Dict]:
    """
    扫描系统可用串口（用于 SLCAN 模式）。
    返回 [{"port": "COM3", "desc": "USB-CAN Adapter"}, ...]
    需要 pyserial，未安装时返回空列表。
    """
    try:
        from serial.tools.list_ports import comports
        return [{"port": p.device, "desc": p.description} for p in comports()]
    except ImportError:
        logger.debug("pyserial 未安装，无法扫描串口")
        return []
    except Exception as e:
        logger.warning("扫描串口失败: %s", e)
        return []
