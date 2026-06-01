"""
多臂管理器 — 统一管理不同 CAN 接口的机械臂实例

提供 Singleton ArmManager，支持：
  - 按名注册 CAN 直连臂
  - 按名获取、遍历、批量断开
  - 从 multi_arm_config.yaml 批量创建
"""

import logging
import threading
from pathlib import Path
from typing import Dict, Optional

import yaml

from el_a3_sdk.protocol import LogLevel

logger = logging.getLogger("el_a3_sdk.arm_manager")


class ArmManager:
    """多臂管理器 — 统一管理不同 CAN 接口的机械臂实例"""

    _instance: Optional["ArmManager"] = None
    _lock = threading.Lock()

    def __new__(cls) -> "ArmManager":
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._arms: Dict[str, object] = {}
            return cls._instance

    @classmethod
    def get_instance(cls) -> "ArmManager":
        return cls()

    @classmethod
    def reset(cls):
        """销毁 Singleton（仅用于测试）"""
        with cls._lock:
            if cls._instance is not None:
                cls._instance.disconnect_all()
                cls._instance = None

    # ------------------------------------------------------------------
    # 注册 / 获取
    # ------------------------------------------------------------------

    def register_can_arm(
        self,
        name: str,
        can_name: str = "can0",
        **kwargs,
    ):
        """注册 CAN 直连模式臂，返回 ELA3Interface 实例

        Args:
            name: 臂的唯一标识（如 "master"）
            can_name: CAN 接口名（如 "can0"）
            **kwargs: 传给 ELA3Interface 的其他参数
        """
        if name in self._arms:
            logger.warning("臂 '%s' 已注册，返回已有实例", name)
            return self._arms[name]

        from el_a3_sdk.interface import ELA3Interface

        arm = ELA3Interface(can_name=can_name, **kwargs)
        self._arms[name] = arm
        logger.info("注册 CAN 臂 '%s' (can=%s)", name, can_name)
        return arm

    def get_arm(self, name: str):
        """按名获取已注册的臂实例"""
        if name not in self._arms:
            raise KeyError(f"臂 '{name}' 未注册")
        return self._arms[name]

    def get_all_arms(self) -> Dict[str, object]:
        return dict(self._arms)

    def has_arm(self, name: str) -> bool:
        return name in self._arms

    def unregister(self, name: str):
        """注销并断开一个臂"""
        arm = self._arms.pop(name, None)
        if arm is not None:
            try:
                arm.DisconnectPort()
            except Exception as e:
                logger.warning("断开 '%s' 时出错: %s", name, e)

    def disconnect_all(self):
        """断开所有臂"""
        for name in list(self._arms.keys()):
            self.unregister(name)

    # ------------------------------------------------------------------
    # 工厂方法：从 config 文件批量创建
    # ------------------------------------------------------------------

    @staticmethod
    def from_config(
        config_path: str,
        auto_connect: bool = False,
    ) -> "ArmManager":
        """从 multi_arm_config.yaml 批量创建臂

        Args:
            config_path: YAML 配置文件路径
            auto_connect: 创建后是否自动调用 ConnectPort
        """
        path = Path(config_path)
        if not path.exists():
            raise FileNotFoundError(f"配置文件不存在: {config_path}")

        with open(path, "r") as f:
            cfg = yaml.safe_load(f)

        mgr = ArmManager.get_instance()
        arms_cfg = cfg.get("arms", {})

        for arm_name, arm_params in arms_cfg.items():
            if not arm_params.get("enabled", True):
                logger.info("跳过已禁用的臂: %s", arm_name)
                continue

            can_iface = arm_params.get("can_interface", "can0")
            host_id = arm_params.get("host_can_id", 0xFD)
            inertia_path = arm_params.get("inertia_config_path")

            extra = {}
            if inertia_path:
                extra["inertia_config_path"] = inertia_path

            arm = mgr.register_can_arm(
                name=arm_name,
                can_name=can_iface,
                host_can_id=host_id,
                **extra,
            )

            if auto_connect:
                arm.ConnectPort()

        return mgr

    # ------------------------------------------------------------------
    # 便捷属性
    # ------------------------------------------------------------------

    @property
    def arm_names(self):
        return list(self._arms.keys())

    def __len__(self):
        return len(self._arms)

    def __contains__(self, name: str):
        return name in self._arms

    def __getitem__(self, name: str):
        return self.get_arm(name)

    def __repr__(self):
        return f"ArmManager(arms={self.arm_names})"
