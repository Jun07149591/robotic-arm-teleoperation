import argparse
import json
import os
import re
import shlex
from dataclasses import dataclass, field
from typing import Dict, Iterable, Optional, Tuple


@dataclass(frozen=True)
class AxisBinding:
    index: Optional[int]
    invert: bool = False

    def read(self, axes) -> float:
        if self.index is None or self.index >= len(axes):
            return 0.0
        value = axes[self.index]
        return -value if self.invert else value


@dataclass(frozen=True)
class TriggerBinding:
    index: Optional[int] = None
    button: Optional[int] = None
    scale: float = 0.5
    offset: float = 0.5
    clamp_min: float = 0.0
    clamp_max: float = 1.0

    def read(self, axes, buttons) -> float:
        if self.button is not None:
            if self.button >= len(buttons):
                return 0.0
            return 1.0 if buttons[self.button] else 0.0

        if self.index is None or self.index >= len(axes):
            return 0.0

        value = axes[self.index] * self.scale + self.offset
        if value < self.clamp_min:
            return self.clamp_min
        if value > self.clamp_max:
            return self.clamp_max
        return value


@dataclass(frozen=True)
class StickMap:
    lx: AxisBinding
    ly: AxisBinding
    rx: AxisBinding
    ry: AxisBinding
    dpad_x: AxisBinding
    dpad_y: AxisBinding
    lt: TriggerBinding
    rt: TriggerBinding


@dataclass(frozen=True)
class ButtonMap:
    south: Optional[int]
    east: Optional[int]
    west: Optional[int]
    north: Optional[int]
    lb: Optional[int]
    rb: Optional[int]
    back: Optional[int]
    start: Optional[int]


@dataclass(frozen=True)
class ControllerProfile:
    profile_id: str
    display_name: str
    description: str
    sticks: StickMap
    buttons: ButtonMap
    default_deadzone: float = 0.15
    match_vid_pid: Tuple[Tuple[str, str], ...] = ()
    match_name_patterns: Tuple[str, ...] = ()


@dataclass(frozen=True)
class ControllerDetection:
    device: str
    resolved_device: str
    name: str
    vendor: str
    product: str
    profile: ControllerProfile
    source: str


def _read_text(path: str) -> str:
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return handle.read().strip()
    except OSError:
        return ""


def _js_sysfs_dir(device: str) -> Optional[str]:
    resolved = os.path.realpath(device)
    name = os.path.basename(resolved)
    if not name.startswith("js"):
        return None
    sysfs_dir = os.path.join("/sys/class/input", name, "device")
    return sysfs_dir if os.path.isdir(sysfs_dir) else None


def read_controller_metadata(device: str) -> Dict[str, str]:
    resolved = os.path.realpath(device)
    sysfs_dir = _js_sysfs_dir(device)
    name = ""
    vendor = ""
    product = ""
    if sysfs_dir:
        name = _read_text(os.path.join(sysfs_dir, "name"))
        vendor = _read_text(os.path.join(sysfs_dir, "id", "vendor")).lower()
        product = _read_text(os.path.join(sysfs_dir, "id", "product")).lower()
    return {
        "device": device,
        "resolved_device": resolved,
        "name": name,
        "vendor": vendor,
        "product": product,
    }


PROFILES: Dict[str, ControllerProfile] = {
    "xbox_default": ControllerProfile(
        profile_id="xbox_default",
        display_name="Xbox (xpad/xinput)",
        description="Standard Xbox layout using Linux xpad-style axes.",
        sticks=StickMap(
            lx=AxisBinding(0),
            ly=AxisBinding(1),
            lt=TriggerBinding(index=2, scale=0.5, offset=0.5),
            rx=AxisBinding(3),
            ry=AxisBinding(4),
            rt=TriggerBinding(index=5, scale=0.5, offset=0.5),
            dpad_x=AxisBinding(6),
            dpad_y=AxisBinding(7),
        ),
        buttons=ButtonMap(
            south=0,
            east=1,
            west=2,
            north=3,
            lb=4,
            rb=5,
            back=6,
            start=7,
        ),
        match_name_patterns=("xbox", "x-input", "xinput", "microsoft"),
    ),
    "zikway_3537_1041": ControllerProfile(
        profile_id="zikway_3537_1041",
        display_name="Zikway HID gamepad",
        description="Generic HID layout detected on VID:PID 3537:1041.",
        sticks=StickMap(
            lx=AxisBinding(0),
            ly=AxisBinding(1),
            lt=TriggerBinding(index=4, scale=0.5, offset=0.5),
            rx=AxisBinding(2),
            ry=AxisBinding(3),
            rt=TriggerBinding(index=5, scale=0.5, offset=0.5),
            dpad_x=AxisBinding(6),
            dpad_y=AxisBinding(7),
        ),
        buttons=ButtonMap(
            south=0,
            east=1,
            west=3,
            north=4,
            lb=6,
            rb=7,
            back=10,
            start=11,
        ),
        match_vid_pid=(("3537", "1041"),),
        match_name_patterns=("zikway",),
    ),
    "generic_hid": ControllerProfile(
        profile_id="generic_hid",
        display_name="Generic HID gamepad",
        description="Fallback layout for 8-axis/16-button HID pads.",
        sticks=StickMap(
            lx=AxisBinding(0),
            ly=AxisBinding(1),
            lt=TriggerBinding(index=4, scale=0.5, offset=0.5),
            rx=AxisBinding(2),
            ry=AxisBinding(3),
            rt=TriggerBinding(index=5, scale=0.5, offset=0.5),
            dpad_x=AxisBinding(6),
            dpad_y=AxisBinding(7),
        ),
        buttons=ButtonMap(
            south=0,
            east=1,
            west=3,
            north=4,
            lb=6,
            rb=7,
            back=10,
            start=11,
        ),
    ),
}


def list_profiles() -> Iterable[ControllerProfile]:
    return PROFILES.values()


def get_profile(profile_id: str) -> ControllerProfile:
    if profile_id not in PROFILES:
        raise KeyError(f"Unknown controller profile: {profile_id}")
    return PROFILES[profile_id]


def detect_controller(device: str, requested_profile: str = "auto") -> ControllerDetection:
    metadata = read_controller_metadata(device)

    if requested_profile != "auto":
        return ControllerDetection(
            device=device,
            resolved_device=metadata["resolved_device"],
            name=metadata["name"],
            vendor=metadata["vendor"],
            product=metadata["product"],
            profile=get_profile(requested_profile),
            source="explicit",
        )

    vendor = metadata["vendor"]
    product = metadata["product"]
    name = metadata["name"].lower()

    for profile in PROFILES.values():
        if (vendor, product) in profile.match_vid_pid:
            return ControllerDetection(
                device=device,
                resolved_device=metadata["resolved_device"],
                name=metadata["name"],
                vendor=vendor,
                product=product,
                profile=profile,
                source="vid_pid",
            )

    for profile in PROFILES.values():
        if any(re.search(pattern, name) for pattern in profile.match_name_patterns):
            return ControllerDetection(
                device=device,
                resolved_device=metadata["resolved_device"],
                name=metadata["name"],
                vendor=vendor,
                product=product,
                profile=profile,
                source="name",
            )

    return ControllerDetection(
        device=device,
        resolved_device=metadata["resolved_device"],
        name=metadata["name"],
        vendor=vendor,
        product=product,
        profile=get_profile("generic_hid"),
        source="fallback",
    )


def _format_detection(detection: ControllerDetection, output_format: str) -> str:
    payload = {
        "device": detection.device,
        "resolved_device": detection.resolved_device,
        "name": detection.name,
        "vendor": detection.vendor,
        "product": detection.product,
        "profile": detection.profile.profile_id,
        "profile_name": detection.profile.display_name,
        "source": detection.source,
    }
    if output_format == "json":
        return json.dumps(payload, ensure_ascii=True)
    if output_format == "shell":
        lines = []
        for key, value in payload.items():
            lines.append(f"CONTROLLER_{key.upper()}={shlex.quote(str(value))}")
        return "\n".join(lines)
    return (
        f"device: {payload['device']}\n"
        f"resolved_device: {payload['resolved_device']}\n"
        f"name: {payload['name'] or 'unknown'}\n"
        f"vendor: {payload['vendor'] or 'unknown'}\n"
        f"product: {payload['product'] or 'unknown'}\n"
        f"profile: {payload['profile']} ({payload['profile_name']})\n"
        f"source: {payload['source']}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Detect joystick type and select a controller profile.")
    parser.add_argument("--device", default="/dev/input/js0", help="Joystick device path (default: /dev/input/js0)")
    parser.add_argument(
        "--profile",
        default="auto",
        choices=["auto", *PROFILES.keys()],
        help="Force a controller profile instead of auto-detection",
    )
    parser.add_argument(
        "--format",
        default="text",
        choices=["text", "json", "shell"],
        help="Output format",
    )
    parser.add_argument("--list-profiles", action="store_true", help="List available profile ids and exit")
    args = parser.parse_args()

    if args.list_profiles:
        for profile in list_profiles():
            print(f"{profile.profile_id}: {profile.display_name} - {profile.description}")
        return 0

    detection = detect_controller(args.device, requested_profile=args.profile)
    print(_format_detection(detection, args.format))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
