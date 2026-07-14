from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field

from src.core.action import Action
from src.core.robot_state import RobotState
from src.utils.logger import get_logger


@dataclass(slots=True)
class MockDriver:
    initial_joint_position: list[float] = field(default_factory=lambda: [0.0] * 6)
    initial_gripper_position: float = 1.0
    logger: logging.Logger = field(init=False, repr=False)
    _connected: bool = field(default=False, init=False, repr=False)
    _state: RobotState = field(init=False, repr=False)
    trajectory: list[Action] = field(default_factory=list, init=False, repr=False)
    master_slave_config: tuple[int, int, int, int] | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        self.logger = get_logger(__name__)
        self._state = RobotState(
            joint_position=list(self.initial_joint_position),
            gripper_position=self.initial_gripper_position,
            is_enabled=False,
            end_pose=[0.0] * 6,
        )

    def connect(self) -> None:
        self._connected = True
        self.logger.info("MockDriver connected")

    def enable(self) -> None:
        self._ensure_connected()
        self._state.is_enabled = True
        self._state.timestamp = time.time()

    def disable(self) -> None:
        self._state.is_enabled = False
        self._state.timestamp = time.time()

    def disconnect(self) -> None:
        self._connected = False
        self._state.is_enabled = False
        self._state.timestamp = time.time()
        self.logger.info("MockDriver disconnected")

    def close(self) -> None:
        self.disconnect()

    def stop(self) -> None:
        self.logger.info("MockDriver stop requested")

    def reset(self) -> None:
        self._state = RobotState(
            joint_position=[0.0] * 6,
            gripper_position=self.initial_gripper_position,
            is_enabled=False,
            end_pose=[0.0] * 6,
        )
        self.trajectory.clear()

    def home(self) -> None:
        self.send_action(
            Action(joint_position=list(self.initial_joint_position), gripper_position=self.initial_gripper_position)
        )

    def get_state(self) -> RobotState:
        self._ensure_connected()
        self._state.timestamp = time.time()
        return RobotState(
            timestamp=self._state.timestamp,
            joint_position=list(self._state.joint_position),
            gripper_position=self._state.gripper_position,
            is_enabled=self._state.is_enabled,
            error_code=self._state.error_code,
            end_pose=list(self._state.end_pose) if self._state.end_pose is not None else None,
            ctrl_mode=self._state.ctrl_mode,
            arm_status=self._state.arm_status,
            mode_feedback=self._state.mode_feedback,
            teach_status=self._state.teach_status,
            motion_status=self._state.motion_status,
            trajectory_num=self._state.trajectory_num,
            joint_limit_flags=list(self._state.joint_limit_flags),
            joint_communication_flags=list(self._state.joint_communication_flags),
        )

    def send_action(self, action: Action) -> None:
        self._ensure_connected()
        if not self._state.is_enabled:
            raise RuntimeError("Mock robot must be enabled before sending actions")
        self._state.joint_position = list(action.joint_position)
        self._state.gripper_position = action.gripper_position
        self._state.end_pose = [
            sum(self._state.joint_position) * 10.0,
            self._state.joint_position[0] * 100.0,
            200.0 + self._state.joint_position[1] * 100.0,
            self._state.joint_position[3],
            self._state.joint_position[4],
            self._state.joint_position[5],
        ]
        self._state.timestamp = time.time()
        self.trajectory.append(action)
        self.logger.info("MockDriver action: joints=%s gripper=%.3f", action.joint_position, action.gripper_position)

    def open_gripper(self) -> None:
        self.send_action(Action(joint_position=list(self._state.joint_position), gripper_position=1.0))

    def close_gripper(self) -> None:
        self.send_action(Action(joint_position=list(self._state.joint_position), gripper_position=0.0))

    def configure_master_slave(
        self,
        linkage_config: int,
        feedback_offset: int = 0x00,
        ctrl_offset: int = 0x00,
        linkage_offset: int = 0x00,
    ) -> None:
        self._ensure_connected()
        self._validate_master_slave_config(linkage_config, feedback_offset, ctrl_offset, linkage_offset)
        self.master_slave_config = (linkage_config, feedback_offset, ctrl_offset, linkage_offset)
        self.logger.info(
            "MockDriver MasterSlaveConfig: linkage=0x%02X feedback=0x%02X ctrl=0x%02X linkage_offset=0x%02X",
            linkage_config,
            feedback_offset,
            ctrl_offset,
            linkage_offset,
        )

    def _validate_master_slave_config(
        self,
        linkage_config: int,
        feedback_offset: int,
        ctrl_offset: int,
        linkage_offset: int,
    ) -> None:
        if linkage_config not in (0xFA, 0xFC):
            raise ValueError("linkage_config must be 0xFA for master or 0xFC for slave")
        for name, value in (
            ("feedback_offset", feedback_offset),
            ("ctrl_offset", ctrl_offset),
            ("linkage_offset", linkage_offset),
        ):
            if value not in (0x00, 0x10, 0x20):
                raise ValueError(f"{name} must be one of 0x00, 0x10, or 0x20")

    def _ensure_connected(self) -> None:
        if not self._connected:
            raise RuntimeError("MockDriver is not connected")
