from __future__ import annotations

from dataclasses import dataclass


@dataclass
class GripperCommand:
    angle: float
    effort: float


@dataclass
class XboxGripperVelocityControl:
    """Velocity-style gripper target controller for gamepad teleop."""

    angle: float = 0.0
    last_sent: float = -1.0
    hold_active: bool = False
    last_hold_time: float = 0.0
    max_angle: float = 2.0
    speed: float = 3.0
    deadzone: float = 0.2
    release_threshold: float = 0.03
    min_send_delta: float = 0.01
    hold_interval: float = 0.25
    close_effort: float = 0.65
    hold_effort: float = 0.65
    feedback_synced: bool = False

    def sync_feedback_angle(self, angle: float) -> bool:
        """Initialize the target from measured gripper position once."""

        if self.feedback_synced or self.last_sent >= 0.0:
            return False
        self.angle = max(0.0, min(self.max_angle, float(angle)))
        self.last_sent = self.angle
        self.feedback_synced = True
        self.hold_active = self.angle > self.release_threshold
        return True

    def update(self, *, dpad_y: float, dt: float, now: float) -> GripperCommand | None:
        """Update gripper target from D-pad Y and return a command to publish.

        D-pad up is negative in the existing Xbox profile, so negative input
        closes the gripper and positive input opens it.
        """

        if abs(dpad_y) > self.deadzone:
            speed = (-float(dpad_y)) * self.speed
            self.angle = max(0.0, min(self.max_angle, self.angle + speed * dt))
            if abs(self.angle - self.last_sent) > self.min_send_delta:
                self.last_sent = self.angle
                self.hold_active = self.angle > self.release_threshold
                self.last_hold_time = now
                effort = self.close_effort if self.hold_active else 0.0
                return GripperCommand(self.angle, effort)
            return None

        if not self.hold_active:
            return None
        if self.angle <= self.release_threshold:
            self.hold_active = False
            return None
        if now - self.last_hold_time >= self.hold_interval:
            self.last_hold_time = now
            return GripperCommand(self.angle, self.hold_effort)
        return None
