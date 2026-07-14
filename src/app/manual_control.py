from __future__ import annotations

import time
from dataclasses import dataclass

from src.core.action import Action
from src.core.executor import ExecutionRecord, Executor
from src.core.robot_state import RobotState


@dataclass(slots=True)
class ManualControl:
    driver: object
    executor: Executor
    home_joint_position: list[float]
    home_gripper_position: float

    def read_state(self) -> RobotState:
        return self.driver.get_state()

    def send_joint_action(self, joint_position: list[float], gripper_position: float | None = None) -> ExecutionRecord:
        state = self.read_state()
        target_gripper = state.gripper_position if gripper_position is None else gripper_position
        return self.executor.execute(Action(joint_position=joint_position, gripper_position=target_gripper), state)

    def move_to_joint_action(self, joint_position: list[float], gripper_position: float | None = None) -> ExecutionRecord:
        final_record: ExecutionRecord | None = None
        started = time.monotonic()
        last_progress_monotonic = started
        last_remaining: float | None = None
        joint_tolerance = 0.01
        gripper_tolerance = 0.01
        for _ in range(180):
            state = self.read_state()
            target_gripper = state.gripper_position if gripper_position is None else gripper_position
            joint_remaining = [abs(target - current) for target, current in zip(joint_position, state.joint_position)]
            gripper_remaining = abs(target_gripper - state.gripper_position)
            remaining = max(joint_remaining)
            if remaining < joint_tolerance and gripper_remaining < gripper_tolerance:
                if final_record is not None:
                    return final_record
                return self.executor.execute(Action(joint_position=joint_position, gripper_position=target_gripper), state)
            if last_remaining is None or remaining < last_remaining - 1e-4:
                last_progress_monotonic = time.monotonic()
            elif time.monotonic() - last_progress_monotonic > 3.0:
                raise RuntimeError("Move did not progress; check robot feedback/CAN state")
            if time.monotonic() - started > 45.0:
                raise RuntimeError("Move timed out before reaching target")
            final_record = self.executor.execute(Action(joint_position=joint_position, gripper_position=target_gripper), state)
            if final_record.safety_result.action is None:
                return final_record
            last_remaining = remaining
        raise RuntimeError("Move exceeded maximum safe step count")

    def set_gripper(self, gripper_position: float) -> ExecutionRecord:
        state = self.read_state()
        return self.executor.execute(
            Action(joint_position=list(state.joint_position), gripper_position=gripper_position),
            state,
        )

    def home(self) -> ExecutionRecord:
        return self.move_to_joint_action(list(self.home_joint_position), self.home_gripper_position)

    def send_cartesian_pose(self, x_mm: float, y_mm: float, z_mm: float, rx_deg: float, ry_deg: float, rz_deg: float) -> None:
        writer = getattr(self.driver, "send_cartesian_pose", None)
        if not callable(writer):
            raise RuntimeError("Cartesian pose control is only available for drivers that implement send_cartesian_pose")
        writer(x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg)

    def configure_master_slave(
        self,
        linkage_config: int,
        feedback_offset: int = 0x00,
        ctrl_offset: int = 0x00,
        linkage_offset: int = 0x00,
    ) -> None:
        writer = getattr(self.driver, "configure_master_slave", None)
        if not callable(writer):
            raise RuntimeError("Master/slave configuration is only available for drivers that implement configure_master_slave")
        writer(linkage_config, feedback_offset, ctrl_offset, linkage_offset)

    def probe_and_configure_sdk_feedback(self) -> dict:
        probe = getattr(self.driver, "probe_and_configure_sdk_feedback", None)
        if not callable(probe):
            raise RuntimeError("SDK feedback probing is only available for Piper drivers")
        return probe()

    def auto_identify_feedback_ids(self) -> dict:
        identify = getattr(self.driver, "auto_identify_feedback_ids", None)
        if not callable(identify):
            raise RuntimeError("Feedback ID auto-identification is only available for Piper drivers")
        return identify()
