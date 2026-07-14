from __future__ import annotations

from dataclasses import dataclass, field

from src.core.action import Action
from src.core.robot_state import RobotState


@dataclass(slots=True)
class SafetyResult:
    action: Action | None
    accepted: bool
    reasons: list[str] = field(default_factory=list)


class SafetyLayer:
    def __init__(
        self,
        joint_min: list[float],
        joint_max: list[float],
        max_delta: list[float],
        gripper_min: float,
        gripper_max: float,
    ) -> None:
        if not all(len(values) == 6 for values in (joint_min, joint_max, max_delta)):
            raise ValueError("joint_min, joint_max, and max_delta must all have length 6")
        self.joint_min = [float(value) for value in joint_min]
        self.joint_max = [float(value) for value in joint_max]
        self.max_delta = [float(value) for value in max_delta]
        self.gripper_min = float(gripper_min)
        self.gripper_max = float(gripper_max)

    def apply(self, action: Action, current_state: RobotState) -> SafetyResult:
        reasons: list[str] = []

        if not current_state.is_enabled:
            return SafetyResult(action=None, accepted=False, reasons=["robot is not enabled"])

        if len(action.joint_position) != 6:
            return SafetyResult(action=None, accepted=False, reasons=["action has invalid joint dimension"])

        safe_joints: list[float] = []
        for index, target in enumerate(action.joint_position):
            lower = self.joint_min[index]
            upper = self.joint_max[index]
            clamped = min(max(target, lower), upper)
            if clamped != target:
                reasons.append(f"joint {index} clamped to limit")

            current = current_state.joint_position[index]
            delta = clamped - current
            max_step = self.max_delta[index]
            if delta > max_step:
                clamped = current + max_step
                reasons.append(f"joint {index} positive delta clamped")
            elif delta < -max_step:
                clamped = current - max_step
                reasons.append(f"joint {index} negative delta clamped")
            safe_joints.append(clamped)

        safe_gripper = min(max(action.gripper_position, self.gripper_min), self.gripper_max)
        if safe_gripper != action.gripper_position:
            reasons.append("gripper position clamped")

        return SafetyResult(
            action=Action(joint_position=safe_joints, gripper_position=safe_gripper),
            accepted=True,
            reasons=reasons,
        )
