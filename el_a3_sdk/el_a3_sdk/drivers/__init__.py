"""CAN driver package entry."""

from el_a3_sdk.drivers.base import CanDriverProtocol, create_can_driver

__all__ = ["CanDriverProtocol", "create_can_driver"]
