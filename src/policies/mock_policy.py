from __future__ import annotations

import math
from dataclasses import dataclass, field

from src.core.action import Action
from src.core.observation import Observation
from src.policies.base_policy import BasePolicy


@dataclass(slots=True)
class MockPolicy(BasePolicy):
    mode: str = "fixed_pose"
    fixed_joint_position: list[float] = field(default_factory=lambda: [0.0, 0.3, 0.5, 0.0, 0.8, 0.0])
    fixed_gripper_position: float = 1.0
    amplitude: float = 0.1
    period_s: float = 4.0
    _step: int = field(default=0, init=False)

    def infer(self, observation: Observation) -> Action:
        self._step += 1
        if self.mode == "fixed_pose":
            return Action(self.fixed_joint_position, self.fixed_gripper_position)
        if self.mode == "sine_scan":
            phase = (self._step / max(self.period_s, 1e-6)) * 2 * math.pi / 10.0
            joints = list(observation.state.joint_position)
            joints[0] = joints[0] + self.amplitude * math.sin(phase)
            joints[2] = joints[2] + self.amplitude * math.cos(phase)
            return Action(joint_position=joints, gripper_position=observation.state.gripper_position)
        if self.mode == "gripper_cycle":
            gripper = 1.0 if self._step % 2 else 0.0
            return Action(joint_position=list(observation.state.joint_position), gripper_position=gripper)
        if self.mode == "scripted_pick_demo":
            phase = self._step % 4
            if phase == 1:
                return Action([0.0, 0.3, 0.4, 0.0, 0.7, 0.0], 1.0)
            if phase == 2:
                return Action([0.1, 0.5, 0.6, 0.0, 0.8, 0.1], 1.0)
            if phase == 3:
                return Action([0.1, 0.5, 0.6, 0.0, 0.8, 0.1], 0.0)
            return Action([0.0, 0.2, 0.3, 0.0, 0.5, 0.0], 0.0)
        raise ValueError(f"Unsupported mock policy mode: {self.mode}")
