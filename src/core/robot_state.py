from __future__ import annotations

import time
from dataclasses import dataclass, field


@dataclass(slots=True)
class RobotState:
    timestamp: float = field(default_factory=time.time)
    joint_position: list[float] = field(default_factory=lambda: [0.0] * 6)
    gripper_position: float = 0.0
    is_enabled: bool = False
    error_code: int | None = None
    end_pose: list[float] | None = None
    ctrl_mode: int = 0
    arm_status: int = 0
    mode_feedback: int = 0
    teach_status: int = 0
    motion_status: int = 0
    trajectory_num: int = 0
    joint_limit_flags: list[bool] = field(default_factory=lambda: [False] * 6)
    joint_communication_flags: list[bool] = field(default_factory=lambda: [False] * 6)

    def __post_init__(self) -> None:
        self.joint_position = [float(value) for value in self.joint_position]
        self.gripper_position = float(self.gripper_position)
        self.ctrl_mode = int(self.ctrl_mode)
        self.arm_status = int(self.arm_status)
        self.mode_feedback = int(self.mode_feedback)
        self.teach_status = int(self.teach_status)
        self.motion_status = int(self.motion_status)
        self.trajectory_num = int(self.trajectory_num)
        self.joint_limit_flags = [bool(value) for value in self.joint_limit_flags]
        self.joint_communication_flags = [bool(value) for value in self.joint_communication_flags]
        if len(self.joint_position) != 6:
            raise ValueError(f"Expected 6 joint values, got {len(self.joint_position)}")
        if len(self.joint_limit_flags) != 6:
            raise ValueError(f"Expected 6 joint limit flags, got {len(self.joint_limit_flags)}")
        if len(self.joint_communication_flags) != 6:
            raise ValueError(
                f"Expected 6 joint communication flags, got {len(self.joint_communication_flags)}"
            )
        if self.end_pose is not None:
            self.end_pose = [float(value) for value in self.end_pose]
            if len(self.end_pose) != 6:
                raise ValueError(f"Expected 6 end pose values, got {len(self.end_pose)}")

    def as_vector(self) -> list[float]:
        return [*self.joint_position, self.gripper_position]
