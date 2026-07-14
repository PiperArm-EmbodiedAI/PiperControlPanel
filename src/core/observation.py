from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from src.core.robot_state import RobotState


@dataclass(slots=True)
class Observation:
    state: RobotState
    images: dict[str, Any | None] = field(
        default_factory=lambda: {
            "cam_high": None,
            "cam_left_wrist": None,
            "cam_right_wrist": None,
        }
    )
    prompt: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "state": {
                "timestamp": self.state.timestamp,
                "joint_position": list(self.state.joint_position),
                "gripper_position": self.state.gripper_position,
                "is_enabled": self.state.is_enabled,
                "error_code": self.state.error_code,
                "ctrl_mode": self.state.ctrl_mode,
                "arm_status": self.state.arm_status,
                "mode_feedback": self.state.mode_feedback,
                "teach_status": self.state.teach_status,
                "motion_status": self.state.motion_status,
                "trajectory_num": self.state.trajectory_num,
                "joint_limit_flags": list(self.state.joint_limit_flags),
                "joint_communication_flags": list(self.state.joint_communication_flags),
            },
            "images": dict(self.images),
            "prompt": self.prompt,
        }
