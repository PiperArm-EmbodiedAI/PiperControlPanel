from __future__ import annotations

import math
from dataclasses import dataclass, field

from src.app.session import RobotSession
from src.core.action import Action
from src.core.executor import ExecutionRecord
from src.core.robot_state import RobotState


@dataclass(slots=True)
class GuiState:
    connected: bool
    enabled: bool
    joint_position: list[float]
    gripper_position: float
    error_code: int | None
    end_pose: list[float] | None
    ctrl_mode: int = 0
    arm_status: int = 0
    mode_feedback: int = 0
    teach_status: int = 0
    motion_status: int = 0
    trajectory_num: int = 0
    joint_limit_flags: list[bool] = field(default_factory=lambda: [False] * 6)
    joint_communication_flags: list[bool] = field(default_factory=lambda: [False] * 6)

    @classmethod
    def from_robot_state(cls, connected: bool, state: RobotState) -> "GuiState":
        return cls(
            connected=connected,
            enabled=state.is_enabled,
            joint_position=list(state.joint_position),
            gripper_position=state.gripper_position,
            error_code=state.error_code,
            end_pose=list(state.end_pose) if state.end_pose is not None else None,
            ctrl_mode=state.ctrl_mode,
            arm_status=state.arm_status,
            mode_feedback=state.mode_feedback,
            teach_status=state.teach_status,
            motion_status=state.motion_status,
            trajectory_num=state.trajectory_num,
            joint_limit_flags=list(state.joint_limit_flags),
            joint_communication_flags=list(state.joint_communication_flags),
        )


@dataclass(slots=True)
class CommandResult:
    message: str
    safety_reasons: list[str] = field(default_factory=list)
    requested_action: Action | None = None
    sent_action: Action | None = None
    sdk_joint_units: list[int] = field(default_factory=list)
    sdk_gripper_units: int | None = None
    cartesian_sdk_units: tuple[int, int, int, int, int, int] | None = None

    @classmethod
    def from_execution(cls, message: str, execution: ExecutionRecord) -> "CommandResult":
        sent_action = execution.safety_result.action
        return cls(
            message=message,
            safety_reasons=list(execution.safety_result.reasons),
            requested_action=execution.requested_action,
            sent_action=sent_action,
            sdk_joint_units=[int(round(math.degrees(value) * 1000.0)) for value in sent_action.joint_position]
            if sent_action is not None
            else [],
            sdk_gripper_units=int(round(sent_action.gripper_position * 70.0 * 1000.0)) if sent_action is not None else None,
        )


class PiperGuiViewModel:
    def __init__(
        self,
        mode: str = "mock",
        robot_config_path: str = "config/robot.yaml",
        policy_config_path: str = "config/policy.yaml",
        demo_config_path: str = "config/demo.yaml",
    ) -> None:
        self.session = RobotSession.from_config(
            mode=mode,
            robot_config_path=robot_config_path,
            policy_config_path=policy_config_path,
            demo_config_path=demo_config_path,
        )

    @property
    def robot_config(self) -> dict:
        return self.session.factory.robot_config

    @property
    def mode(self) -> str:
        return self.session.mode

    def connect(self) -> CommandResult:
        self.session.connect()
        return CommandResult("connected")

    def disconnect(self) -> CommandResult:
        self.session.disconnect()
        return CommandResult("disconnected")

    def enable(self) -> CommandResult:
        self.session.enable()
        return CommandResult("enabled")

    def disable(self) -> CommandResult:
        self.session.disable()
        return CommandResult("disabled")

    def stop(self) -> CommandResult:
        self.session.stop()
        return CommandResult("stop sent")

    def reset_fault(self) -> CommandResult:
        self.session.reset()
        return CommandResult("reset fault sent")

    def home(self) -> CommandResult:
        execution = self.session.home()
        return CommandResult.from_execution("home target reached", execution)

    def send_joints_degrees(self, joints_deg: list[float], gripper: float, move_to_target: bool = True) -> CommandResult:
        joints_rad = [math.radians(value) for value in joints_deg]
        if move_to_target:
            execution = self.session.move_to_joint_action(joints_rad, gripper)
        else:
            execution = self.session.send_joint_action(joints_rad, gripper)
        return CommandResult.from_execution("target sent", execution)

    def set_gripper(self, gripper: float) -> CommandResult:
        execution = self.session.set_gripper(gripper)
        return CommandResult.from_execution("gripper target sent", execution)

    def send_cartesian_pose(self, x_mm: float, y_mm: float, z_mm: float, rx_deg: float, ry_deg: float, rz_deg: float) -> CommandResult:
        if not self.robot_config.get("gui", {}).get("experimental_cartesian_enabled", False):
            return CommandResult("Cartesian send is disabled. Set robot.gui.experimental_cartesian_enabled=true after validating the workspace.")
        self.session.send_cartesian_pose(x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg)
        return CommandResult(
            message="cartesian target sent",
            cartesian_sdk_units=(
                int(round(x_mm * 1000.0)),
                int(round(y_mm * 1000.0)),
                int(round(z_mm * 1000.0)),
                int(round(rx_deg * 1000.0)),
                int(round(ry_deg * 1000.0)),
                int(round(rz_deg * 1000.0)),
            ),
        )

    def configure_master_slave(
        self,
        role: str,
        feedback_offset: int = 0x00,
        ctrl_offset: int = 0x00,
        linkage_offset: int = 0x00,
    ) -> CommandResult:
        gui_config = self.robot_config.get("gui", {})
        if not gui_config.get("master_slave_config_enabled", False):
            return CommandResult(
                "Master/slave configuration is disabled. Set robot.gui.master_slave_config_enabled=true after confirming the deployment."
            )
        linkage_config = self._master_slave_role_to_linkage_config(role)
        self._validate_master_slave_offsets(feedback_offset, ctrl_offset, linkage_offset)
        if any(value != 0x00 for value in (feedback_offset, ctrl_offset, linkage_offset)) and not gui_config.get(
            "master_slave_advanced_offsets_enabled", False
        ):
            return CommandResult(
                "Advanced master/slave offsets are disabled. Use 0x00 offsets or set robot.gui.master_slave_advanced_offsets_enabled=true."
            )
        self.session.configure_master_slave(linkage_config, feedback_offset, ctrl_offset, linkage_offset)
        role_label = "master" if linkage_config == 0xFA else "slave"
        return CommandResult(
            "MasterSlaveConfig applied: "
            f"role={role_label} linkage_config=0x{linkage_config:02X} "
            f"feedback_offset=0x{feedback_offset:02X} ctrl_offset=0x{ctrl_offset:02X} "
            f"linkage_offset=0x{linkage_offset:02X}"
        )

    def read_state(self) -> GuiState:
        return GuiState.from_robot_state(self.session.connected, self.session.read_state())

    def close(self) -> None:
        self.session.close()

    def _master_slave_role_to_linkage_config(self, role: str) -> int:
        if role == "master":
            return 0xFA
        if role == "slave":
            return 0xFC
        raise ValueError("role must be 'master' or 'slave'")

    def _validate_master_slave_offsets(self, feedback_offset: int, ctrl_offset: int, linkage_offset: int) -> None:
        for name, value in (
            ("feedback_offset", feedback_offset),
            ("ctrl_offset", ctrl_offset),
            ("linkage_offset", linkage_offset),
        ):
            if value not in (0x00, 0x10, 0x20):
                raise ValueError(f"{name} must be one of 0x00, 0x10, or 0x20")
