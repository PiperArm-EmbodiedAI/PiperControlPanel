from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable


@dataclass(slots=True)
class Action:
    joint_position: list[float] = field(default_factory=lambda: [0.0] * 6)
    gripper_position: float = 0.0

    def __post_init__(self) -> None:
        self.joint_position = [float(value) for value in self.joint_position]
        self.gripper_position = float(self.gripper_position)
        self.validate()

    def validate(self) -> None:
        if len(self.joint_position) != 6:
            raise ValueError(f"Expected 6 joint values, got {len(self.joint_position)}")

    @classmethod
    def from_iterables(cls, joint_position: Iterable[float], gripper_position: float) -> "Action":
        return cls(joint_position=list(joint_position), gripper_position=gripper_position)
