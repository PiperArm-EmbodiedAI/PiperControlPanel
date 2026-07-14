from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from typing import Any

from src.core.action import Action
from src.core.robot_state import RobotState
from src.utils.logger import get_logger


@dataclass(slots=True)
class PiperDriver:
    can_interface: str = "can0"
    auto_enable: bool = False
    sdk_client: Any | None = None
    state_reader: Any | None = None
    action_writer: Any | None = None
    gripper_open_value: float = 1.0
    gripper_closed_value: float = 0.0
    gripper_range_mm: float = 70.0
    gripper_effort_newton_meter: float = 1.0
    move_speed_rate: int = 30
    logger: logging.Logger = field(init=False, repr=False)
    _connected: bool = field(default=False, init=False, repr=False)
    _enabled: bool = field(default=False, init=False, repr=False)

    def __post_init__(self) -> None:
        self.logger = get_logger(__name__)

    def connect(self) -> None:
        if self.sdk_client is None:
            from piper_sdk import C_PiperInterface_V2

            self.sdk_client = C_PiperInterface_V2(can_name=self.can_interface, can_auto_init=False)
        create_can_bus = getattr(self.sdk_client, "CreateCanBus", None)
        connect_port = getattr(self.sdk_client, "ConnectPort", None)
        connect = getattr(self.sdk_client, "connect", None)
        if callable(create_can_bus):
            create_can_bus(self.can_interface, "socketcan", 1000000, False)
        if callable(connect_port):
            connect_port(can_init=False, piper_init=True, start_thread=True)
        elif callable(connect):
            connect()
        else:
            raise RuntimeError("No compatible connect method found on Piper SDK client")
        self._connected = True
        if self.auto_enable:
            self.enable()

    def enable(self) -> None:
        self._ensure_connected()
        enable_arm = getattr(self.sdk_client, "EnableArm", None)
        enable = getattr(self.sdk_client, "enable", None)
        if callable(enable_arm):
            enable_arm(7, 0x02)
        elif callable(enable):
            enable()
        else:
            raise RuntimeError("No compatible enable method found on Piper SDK client")
        self._enabled = True

    def disable(self) -> None:
        self._ensure_connected()
        disable_arm = getattr(self.sdk_client, "DisableArm", None)
        disable = getattr(self.sdk_client, "disable", None)
        if callable(disable_arm):
            disable_arm(7, 0x01)
        elif callable(disable):
            disable()
        else:
            raise RuntimeError("No compatible disable method found on Piper SDK client")
        self._enabled = False

    def disconnect(self) -> None:
        if self.sdk_client is not None:
            disconnect_port = getattr(self.sdk_client, "DisconnectPort", None)
            disconnect = getattr(self.sdk_client, "disconnect", None)
            close = getattr(self.sdk_client, "close", None)
            if callable(disconnect_port):
                disconnect_port()
            elif callable(disconnect):
                disconnect()
            elif callable(close):
                close()
        self._connected = False
        self._enabled = False

    def close(self) -> None:
        self.disconnect()

    def home(self) -> None:
        self._ensure_connected()
        search_names = ("GoHome", "go_home", "Home", "home")
        for name in search_names:
            home_method = getattr(self.sdk_client, name, None)
            if callable(home_method):
                home_method()
                return
        raise RuntimeError("No compatible home method found on Piper SDK client")

    def stop(self) -> None:
        self._ensure_connected()
        emergency_stop = getattr(self.sdk_client, "EmergencyStop", None)
        stop = getattr(self.sdk_client, "stop", None)
        if callable(emergency_stop):
            emergency_stop(0x01)
        elif callable(stop):
            stop()
        else:
            raise RuntimeError("No compatible stop method found on Piper SDK client")

    def start_drag_teach(self) -> None:
        self._send_drag_teach_control(0x01)

    def stop_drag_teach(self) -> None:
        self._send_drag_teach_control(0x02)

    def reset(self) -> None:
        self._ensure_connected()
        emergency_stop = getattr(self.sdk_client, "EmergencyStop", None)
        reset_piper = getattr(self.sdk_client, "ResetPiper", None)
        reset = getattr(self.sdk_client, "reset", None)
        if callable(emergency_stop):
            emergency_stop(0x02)
        if callable(reset_piper):
            reset_piper()
            return
        if callable(reset):
            reset()
            return
        raise RuntimeError("No compatible reset method found on Piper SDK client")

    def get_state(self) -> RobotState:
        self._ensure_connected()
        raw_state = self._read_raw_state()
        return self._normalize_state(raw_state)

    def send_action(self, action: Action) -> None:
        self._ensure_connected()
        if not self._hardware_enabled():
            raise RuntimeError("Robot must be enabled before sending actions")
        if self.action_writer is not None:
            self.action_writer(action)
            return
        motion_ctrl = getattr(self.sdk_client, "MotionCtrl_2", None)
        joint_ctrl = getattr(self.sdk_client, "JointCtrl", None)
        gripper_ctrl = getattr(self.sdk_client, "GripperCtrl", None)
        writer = getattr(self.sdk_client, "send_action", None)
        if callable(motion_ctrl) and callable(joint_ctrl):
            motion_ctrl(0x01, 0x01, self.move_speed_rate, 0x00, 0, 0x00)
            joint_ctrl(*[self._radians_to_sdk_units(value) for value in action.joint_position])
            if callable(gripper_ctrl):
                gripper_ctrl(
                    gripper_angle=self._gripper_to_sdk_units(action.gripper_position),
                    gripper_effort=self._gripper_effort_to_sdk_units(self.gripper_effort_newton_meter),
                    gripper_code=0x01,
                    set_zero=0x00,
                )
            return
        if callable(writer):
            writer(action.joint_position, action.gripper_position)
            return
        raise RuntimeError("No action writer available for Piper driver")

    def send_joint_positions(self, joint_position: list[float] | tuple[float, ...]) -> None:
        self._ensure_connected()
        if not self._hardware_enabled():
            raise RuntimeError("Robot must be enabled before sending joint positions")
        if len(joint_position) != 6 or any(not math.isfinite(float(value)) for value in joint_position):
            raise ValueError("Joint position must contain six finite values")
        motion_ctrl = getattr(self.sdk_client, "MotionCtrl_2", None)
        joint_ctrl = getattr(self.sdk_client, "JointCtrl", None)
        if not callable(motion_ctrl) or not callable(joint_ctrl):
            raise RuntimeError("No joint position writer available for Piper driver")
        motion_ctrl(0x01, 0x01, self.move_speed_rate, 0x00, 0, 0x00)
        joint_ctrl(*[self._radians_to_sdk_units(value) for value in joint_position])

    def send_cartesian_pose(self, x_mm: float, y_mm: float, z_mm: float, rx_deg: float, ry_deg: float, rz_deg: float) -> None:
        self._ensure_connected()
        if not self._hardware_enabled():
            raise RuntimeError("Robot must be enabled before sending cartesian pose")
        values = (x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg)
        if any(not math.isfinite(float(value)) for value in values):
            raise ValueError("Cartesian pose values must be finite")
        sdk_values = tuple(int(round(float(value) * 1000.0)) for value in values)
        mode_ctrl = getattr(self.sdk_client, "ModeCtrl", None)
        motion_ctrl = getattr(self.sdk_client, "MotionCtrl_2", None)
        end_pose_ctrl = getattr(self.sdk_client, "EndPoseCtrl", None)
        if not callable(end_pose_ctrl) or (not callable(mode_ctrl) and not callable(motion_ctrl)):
            raise RuntimeError("No cartesian pose writer available for Piper driver")
        if callable(mode_ctrl):
            mode_ctrl(0x01, 0x00, self.move_speed_rate, 0x00)
        else:
            motion_ctrl(0x01, 0x00, self.move_speed_rate, 0x00, 0, 0x00)
        end_pose_ctrl(*sdk_values)

    def set_gripper_position(self, position: float) -> None:
        self._ensure_connected()
        if not self._hardware_enabled():
            raise RuntimeError("Robot must be enabled before setting gripper")
        gripper_ctrl = getattr(self.sdk_client, "GripperCtrl", None)
        if not callable(gripper_ctrl):
            raise RuntimeError("No gripper writer available for Piper driver")
        gripper_ctrl(
            gripper_angle=self._gripper_to_sdk_units(position),
            gripper_effort=self._gripper_effort_to_sdk_units(self.gripper_effort_newton_meter),
            gripper_code=0x01,
            set_zero=0x00,
        )

    def configure_master_slave(
        self,
        linkage_config: int,
        feedback_offset: int = 0x00,
        ctrl_offset: int = 0x00,
        linkage_offset: int = 0x00,
    ) -> None:
        self._ensure_connected()
        self._validate_master_slave_config(linkage_config, feedback_offset, ctrl_offset, linkage_offset)
        master_slave_config = getattr(self.sdk_client, "MasterSlaveConfig", None)
        if not callable(master_slave_config):
            raise RuntimeError("Piper SDK client does not provide MasterSlaveConfig")
        master_slave_config(linkage_config, feedback_offset, ctrl_offset, linkage_offset)

    def probe_and_configure_sdk_feedback(self) -> dict[str, Any]:
        self._ensure_connected()
        before = self._direct_sdk_feedback_snapshot()
        configured = False
        config_error = None
        master_slave_config = getattr(self.sdk_client, "MasterSlaveConfig", None)
        if callable(master_slave_config):
            try:
                master_slave_config(0xFC, 0x00, 0x00, 0x00)
                configured = True
                time.sleep(0.5)
            except Exception as exc:
                config_error = str(exc)
        after = self._direct_sdk_feedback_snapshot()
        return {
            "configured_default_output": configured,
            "config_error": config_error,
            "before": before,
            "after": after,
        }

    def auto_identify_feedback_ids(self) -> dict[str, Any]:
        self._ensure_connected()
        before_ids = self._capture_feedback_id_counts()
        probe = self.probe_and_configure_sdk_feedback()
        after_ids = self._capture_feedback_id_counts()
        return {
            "before_ids": before_ids,
            "after_ids": after_ids,
            "detected_before": self._classify_feedback_ids(before_ids),
            "detected_after": self._classify_feedback_ids(after_ids),
            "direct_probe": probe,
        }

    def open_gripper(self) -> None:
        state = self.get_state()
        self.send_action(Action(joint_position=state.joint_position, gripper_position=self.gripper_open_value))

    def close_gripper(self) -> None:
        state = self.get_state()
        self.send_action(Action(joint_position=state.joint_position, gripper_position=self.gripper_closed_value))

    def _capture_feedback_id_counts(self, seconds: float = 0.5) -> dict[str, int]:
        try:
            import can
        except ModuleNotFoundError as exc:
            raise RuntimeError("python-can is required to identify feedback IDs") from exc
        bus = can.interface.Bus(channel=self.can_interface, interface="socketcan")
        counts: dict[int, int] = {}
        deadline = time.monotonic() + max(float(seconds), 0.1)
        try:
            while time.monotonic() < deadline:
                msg = bus.recv(timeout=0.05)
                if msg is None:
                    continue
                arbitration_id = int(msg.arbitration_id)
                counts[arbitration_id] = counts.get(arbitration_id, 0) + 1
        finally:
            shutdown = getattr(bus, "shutdown", None)
            if callable(shutdown):
                shutdown()
        return {f"0x{arbitration_id:X}": count for arbitration_id, count in sorted(counts.items())}

    def _classify_feedback_ids(self, counts: dict[str, int]) -> dict[str, Any]:
        seen = {int(key, 16) for key in counts}
        default_endpose = {0x2A2, 0x2A3, 0x2A4}
        default_joint = {0x2A5, 0x2A6, 0x2A7}
        offset_2b = set(range(0x2B1, 0x2B9))
        offset_2c = set(range(0x2C1, 0x2C9))
        high_spd = set(range(0x251, 0x257))
        low_spd = set(range(0x261, 0x267))
        return {
            "default_endpose_complete": default_endpose <= seen,
            "default_joint_complete": default_joint <= seen,
            "offset_2b_seen": bool(seen & offset_2b),
            "offset_2c_seen": bool(seen & offset_2c),
            "high_speed_driver_feedback_complete": high_spd <= seen,
            "low_speed_driver_feedback_complete": low_spd <= seen,
            "ids_seen": sorted(counts),
        }

    def _direct_sdk_feedback_snapshot(self) -> dict[str, Any]:
        joint_feedback = self._call_optional_sdk("GetArmJointMsgs")
        endpose_feedback = self._call_optional_sdk("GetArmEndPoseMsgs")
        status_feedback = self._call_optional_sdk("GetArmStatus")
        joint_state = getattr(joint_feedback, "joint_state", joint_feedback)
        end_pose = getattr(endpose_feedback, "end_pose", endpose_feedback)
        joint_values = self._sdk_joint_values(joint_state)
        endpose_values = self._sdk_endpose_values(end_pose)
        status_timestamp = self._feedback_timestamp(status_feedback) if status_feedback is not None else 0.0
        return {
            "joint_values": joint_values,
            "joint_nonzero": any(value != 0 for value in joint_values),
            "endpose_values": endpose_values,
            "endpose_nonzero": any(value != 0 for value in endpose_values),
            "status_timestamp": status_timestamp,
        }

    def _call_optional_sdk(self, name: str) -> Any | None:
        method = getattr(self.sdk_client, name, None)
        return method() if callable(method) else None

    def _sdk_joint_values(self, joint_state: Any | None) -> list[int]:
        if joint_state is None:
            return [0] * 6
        values = []
        for index in range(1, 7):
            values.append(int(getattr(joint_state, f"joint_{index}", 0)))
        return values

    def _sdk_endpose_values(self, end_pose: Any | None) -> list[int]:
        if end_pose is None:
            return [0] * 6
        return [int(getattr(end_pose, name, 0)) for name in ("X_axis", "Y_axis", "Z_axis", "RX_axis", "RY_axis", "RZ_axis")]

    def _read_raw_state(self) -> Any:
        if self.state_reader is not None:
            return self.state_reader()
        get_joint_msgs = getattr(self.sdk_client, "GetArmJointMsgs", None)
        if callable(get_joint_msgs):
            get_gripper_msgs = getattr(self.sdk_client, "GetArmGripperMsgs", None)
            get_status = getattr(self.sdk_client, "GetArmStatus", None)
            get_end_pose = getattr(self.sdk_client, "GetArmEndPoseMsgs", None)
            get_enable_status = getattr(self.sdk_client, "GetArmEnableStatus", None)
            joint_feedback = get_joint_msgs()
            status_feedback = get_status() if callable(get_status) else None
            end_pose_feedback = get_end_pose() if callable(get_end_pose) else None
            return {
                "timestamp": self._feedback_timestamp(
                    joint_feedback,
                    status_feedback,
                    end_pose_feedback,
                ),
                "joint_feedback": joint_feedback,
                "gripper_feedback": get_gripper_msgs() if callable(get_gripper_msgs) else None,
                "status_feedback": status_feedback,
                "end_pose_feedback": end_pose_feedback,
                "enable_feedback": get_enable_status() if callable(get_enable_status) else None,
            }
        reader = getattr(self.sdk_client, "get_state", None)
        if callable(reader):
            return reader()
        raise RuntimeError("No state reader available for Piper driver")

    def _normalize_state(self, raw_state: Any) -> RobotState:
        if isinstance(raw_state, RobotState):
            return raw_state
        if isinstance(raw_state, dict) and "joint_feedback" in raw_state:
            joint_feedback = raw_state["joint_feedback"]
            gripper_feedback = raw_state.get("gripper_feedback")
            status_feedback = raw_state.get("status_feedback")
            end_pose_feedback = raw_state.get("end_pose_feedback")
            enable_feedback = raw_state.get("enable_feedback")
            joint_state = getattr(joint_feedback, "joint_state", joint_feedback)
            gripper_state = getattr(gripper_feedback, "gripper_state", gripper_feedback)
            end_pose_state = getattr(end_pose_feedback, "end_pose", end_pose_feedback)
            joint_position = [
                self._sdk_units_to_radians(getattr(joint_state, f"joint_{index}"))
                for index in range(1, 7)
            ]
            gripper_units = getattr(gripper_state, "grippers_angle", 0) if gripper_state is not None else 0
            return RobotState(
                timestamp=float(raw_state.get("timestamp", time.time())),
                joint_position=joint_position,
                gripper_position=self._sdk_gripper_to_normalized(gripper_units),
                is_enabled=self._normalize_enable_feedback(enable_feedback),
                end_pose=self._normalize_end_pose(end_pose_state),
                **self._normalize_status_feedback(status_feedback),
            )
        if isinstance(raw_state, dict):
            return RobotState(
                timestamp=float(raw_state.get("timestamp", time.time())),
                joint_position=list(raw_state.get("joint_position", [0.0] * 6)),
                gripper_position=float(raw_state.get("gripper_position", 0.0)),
                is_enabled=bool(raw_state.get("is_enabled", self._enabled)),
                error_code=raw_state.get("error_code"),
                end_pose=raw_state.get("end_pose"),
                ctrl_mode=raw_state.get("ctrl_mode", 0),
                arm_status=raw_state.get("arm_status", 0),
                mode_feedback=raw_state.get("mode_feedback", raw_state.get("mode_feed", 0)),
                teach_status=raw_state.get("teach_status", 0),
                motion_status=raw_state.get("motion_status", 0),
                trajectory_num=raw_state.get("trajectory_num", 0),
                joint_limit_flags=list(raw_state.get("joint_limit_flags", [False] * 6)),
                joint_communication_flags=list(
                    raw_state.get("joint_communication_flags", [False] * 6)
                ),
            )
        joint_position = getattr(raw_state, "joint_position", [0.0] * 6)
        gripper_position = getattr(raw_state, "gripper_position", 0.0)
        is_enabled = getattr(raw_state, "is_enabled", self._enabled)
        error_code = getattr(raw_state, "error_code", None)
        end_pose = getattr(raw_state, "end_pose", None)
        timestamp = getattr(raw_state, "timestamp", time.time())
        return RobotState(
            timestamp=float(timestamp),
            joint_position=list(joint_position),
            gripper_position=float(gripper_position),
            is_enabled=bool(is_enabled),
            error_code=error_code,
            end_pose=end_pose,
            ctrl_mode=getattr(raw_state, "ctrl_mode", 0),
            arm_status=getattr(raw_state, "arm_status", 0),
            mode_feedback=getattr(raw_state, "mode_feedback", getattr(raw_state, "mode_feed", 0)),
            teach_status=getattr(raw_state, "teach_status", 0),
            motion_status=getattr(raw_state, "motion_status", 0),
            trajectory_num=getattr(raw_state, "trajectory_num", 0),
            joint_limit_flags=list(getattr(raw_state, "joint_limit_flags", [False] * 6)),
            joint_communication_flags=list(
                getattr(raw_state, "joint_communication_flags", [False] * 6)
            ),
        )

    def _feedback_timestamp(self, *feedback_values: Any | None) -> float:
        timestamps = []
        for feedback in feedback_values:
            value = getattr(feedback, "time_stamp", None)
            if value is None:
                value = getattr(feedback, "timestamp", None)
            if value is None:
                continue
            parsed = float(value)
            if not math.isfinite(parsed) or parsed <= 0.0:
                return 0.0
            timestamps.append(parsed)
        return min(timestamps) if timestamps else 0.0

    def _normalize_enable_feedback(self, enable_feedback: Any | None) -> bool:
        if enable_feedback is None:
            return self._enabled
        try:
            values = list(enable_feedback)
        except TypeError:
            return False
        return len(values) == 6 and all(bool(value) for value in values)

    def _hardware_enabled(self) -> bool:
        get_enable_status = getattr(self.sdk_client, "GetArmEnableStatus", None)
        if not callable(get_enable_status):
            return self._enabled
        return self._enabled and self._normalize_enable_feedback(get_enable_status())

    def _normalize_status_feedback(self, status_feedback: Any | None) -> dict[str, Any]:
        status_state = status_feedback
        nested_status = getattr(status_feedback, "arm_status", None)
        if nested_status is not None:
            status_state = nested_status
        if status_state is None:
            return {}

        err_status = getattr(status_state, "err_status", None)
        return {
            "error_code": self._status_int(status_state, "err_code", default=None),
            "ctrl_mode": self._status_int(status_state, "ctrl_mode"),
            "arm_status": self._status_int(status_state, "arm_status"),
            "mode_feedback": self._status_int(status_state, "mode_feed", "mode_feedback"),
            "teach_status": self._status_int(status_state, "teach_status"),
            "motion_status": self._status_int(status_state, "motion_status"),
            "trajectory_num": self._status_int(status_state, "trajectory_num"),
            "joint_limit_flags": [
                bool(getattr(err_status, f"joint_{index}_angle_limit", False))
                for index in range(1, 7)
            ],
            "joint_communication_flags": [
                bool(getattr(err_status, f"communication_status_joint_{index}", False))
                for index in range(1, 7)
            ],
        }

    def _status_int(
        self,
        status_state: Any,
        *attribute_names: str,
        default: int | None = 0,
    ) -> int | None:
        for attribute_name in attribute_names:
            value = getattr(status_state, attribute_name, None)
            if value is not None:
                return int(value)
        return default

    def _normalize_end_pose(self, end_pose_state: Any | None) -> list[float] | None:
        if end_pose_state is None:
            return None
        values = []
        for name in ("X_axis", "Y_axis", "Z_axis", "RX_axis", "RY_axis", "RZ_axis"):
            if not hasattr(end_pose_state, name):
                return None
            raw_value = getattr(end_pose_state, name)
            if name in ("X_axis", "Y_axis", "Z_axis"):
                values.append(float(raw_value) / 1000.0)
            else:
                values.append(math.radians(float(raw_value) / 1000.0))
        return values

    def _radians_to_sdk_units(self, radians_value: float) -> int:
        return int(round(math.degrees(radians_value) * 1000.0))

    def _sdk_units_to_radians(self, sdk_value: int) -> float:
        return math.radians(float(sdk_value) / 1000.0)

    def _gripper_to_sdk_units(self, normalized_position: float) -> int:
        clamped = min(max(float(normalized_position), 0.0), 1.0)
        return int(round(clamped * self.gripper_range_mm * 1000.0))

    def _sdk_gripper_to_normalized(self, sdk_value: int) -> float:
        if self.gripper_range_mm <= 0:
            return 0.0
        normalized = float(sdk_value) / (self.gripper_range_mm * 1000.0)
        return min(max(normalized, 0.0), 1.0)

    def _gripper_effort_to_sdk_units(self, effort_value: float) -> int:
        clamped = min(max(float(effort_value), 0.0), 5.0)
        return int(round(clamped * 1000.0))

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

    def _send_drag_teach_control(self, control: int) -> None:
        self._ensure_connected()
        motion_ctrl = getattr(self.sdk_client, "MotionCtrl_1", None)
        if not callable(motion_ctrl):
            raise RuntimeError("Piper SDK client does not provide MotionCtrl_1")
        motion_ctrl(
            emergency_stop=0x00,
            track_ctrl=0x00,
            grag_teach_ctrl=control,
        )

    def _ensure_connected(self) -> None:
        if not self._connected:
            raise RuntimeError(f"Piper driver is not connected on interface {self.can_interface}")
