"""
SLCAN 连接验证脚本 (Windows / 跨平台)

通过 SLCAN USB-CAN 适配器连接 EL-A3 机械臂，验证通信是否正常。

用法:
    python slcan_test.py                     # 默认 COM3, 串口 2Mbps
    python slcan_test.py --port COM5         # 指定串口
    python slcan_test.py --port /dev/ttyACM0 # Linux 下的 SLCAN 设备

依赖:
    pip install pyserial
    或
    pip install el_a3_sdk[slcan]
"""

import argparse
import sys
import time
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from el_a3_sdk import ELA3Interface, LogLevel


def main():
    parser = argparse.ArgumentParser(description="EL-A3 SLCAN 连接验证")
    parser.add_argument("--port", default="COM3",
                        help="串口名 (Windows: COM3, Linux: /dev/ttyACM0)")
    parser.add_argument("--serial-baudrate", type=int, default=2000000,
                        help="串口通信波特率 (默认 2000000)")
    parser.add_argument("--can-bitrate", type=int, default=1000000,
                        help="CAN 总线波特率 (默认 1000000)")
    args = parser.parse_args()

    print(f"=== EL-A3 SLCAN 连接验证 ===")
    print(f"串口:        {args.port}")
    print(f"串口波特率:  {args.serial_baudrate} bps")
    print(f"CAN 波特率:  {args.can_bitrate} bps")
    print()

    arm = ELA3Interface(
        can_name=args.port,
        backend="slcan",
        serial_baudrate=args.serial_baudrate,
        can_bitrate=args.can_bitrate,
        logger_level=LogLevel.INFO,
    )

    print("[1/5] 连接串口...")
    if not arm.ConnectPort():
        print("  连接失败！请检查:")
        print(f"    - 串口 {args.port} 是否存在")
        print("    - USB-CAN 适配器是否已插入")
        print("    - 其他程序是否占用了该串口")
        return 1

    print("  连接成功")
    print()

    print("[2/5] 等待电机反馈 (2 秒)...")
    time.sleep(2.0)

    print("[3/5] 读取关节角度...")
    joints = arm.GetArmJointMsgs()
    print(f"  关节角度 (rad): {[f'{v:.4f}' for v in joints.to_list()]}")
    print(f"  时间戳: {joints.timestamp:.3f}")

    if joints.timestamp == 0:
        print("  未收到电机反馈！请检查:")
        print("    - CAN 总线是否连接到电机")
        print("    - CAN 波特率是否为 1Mbps")
        print("    - 电机是否上电")
    print()

    print("[4/5] CAN 帧率统计...")
    fps = arm.GetCanFps()
    tx_ok, tx_fail, tx_fail_rate = arm.GetCanTxStats()
    bus_state = arm.GetCanBusState()
    print(f"  接收帧率: {fps:.1f} fps")
    print(f"  TX 成功: {tx_ok}, 失败: {tx_fail}, 失败率: {tx_fail_rate:.2%}")
    print(f"  总线状态: {bus_state}")
    print()

    print("[5/5] 查询固件版本...")
    versions = arm.GetAllFirmwareVersions()
    if versions:
        for mid, ver in sorted(versions.items()):
            print(f"  电机 {mid}: {ver.version_str}")
    else:
        print("  未能读取固件版本 (可能需要先使能电机)")
    print()

    arm.DisconnectPort()
    print("=== 验证完成 ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
