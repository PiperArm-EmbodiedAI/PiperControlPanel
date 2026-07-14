from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from threading import RLock
from typing import Any

from src.app.factory import AppFactory
from src.app.manual_control import ManualControl
from src.core.controller import Controller
from src.core.executor import ExecutionRecord
from src.core.robot_state import RobotState


@dataclass(slots=True)
class RobotSession:
    factory: AppFactory
    mode: str = "mock"
    driver: Any = field(init=False)
    executor: Any = field(init=False)
    camera_driver: Any = field(init=False)
    policy: Any = field(init=False)
    manual_control: ManualControl = field(init=False)
    _lock: RLock = field(default_factory=RLock, init=False, repr=False)
    _connected: bool = field(default=False, init=False)

    def __post_init__(self) -> None:
        self.mode = self.factory.resolve_mode(self.mode)
        self.driver = self.factory.build_driver(self.mode)
        self.executor = self.factory.build_executor(self.driver)
        self.camera_driver = self.factory.build_camera()
        self.policy = self.factory.build_policy()
        self.manual_control = ManualControl(
            driver=self.driver,
            executor=self.executor,
            home_joint_position=self.factory.robot_config.get("home_joint_position", [0.0] * 6),
            home_gripper_position=self.factory.robot_config["gripper"]["open"],
        )

    @classmethod
    def from_config(
        cls,
        mode: str | None = None,
        robot_config_path: str = "config/robot.yaml",
        policy_config_path: str = "config/policy.yaml",
        demo_config_path: str = "config/demo.yaml",
    ) -> "RobotSession":
        factory = AppFactory(robot_config_path, policy_config_path, demo_config_path)
        return cls(factory=factory, mode=factory.resolve_mode(mode))

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        with self._lock:
            if self._connected:
                return
            self.driver.connect()
            self._connected = True

    def disconnect(self) -> None:
        with self._lock:
            shutdown = self.factory.robot_config.get("shutdown", {})
            pending_error: Exception | None = None
            try:
                if self._connected and shutdown.get("stop_on_exit", False):
                    self.driver.stop()
                elif self._connected and shutdown.get("auto_disable", True):
                    self._park_and_disable()
            except Exception as exc:
                pending_error = exc
            finally:
                close = getattr(self.driver, "close", None)
                disconnect = getattr(self.driver, "disconnect", None)
                if callable(close):
                    close()
                elif callable(disconnect):
                    disconnect()
                self._connected = False
            if pending_error is not None:
                raise pending_error

    def close(self) -> None:
        self.disconnect()

    def enable(self) -> None:
        with self._lock:
            self.driver.enable()

    def disable(self) -> None:
        with self._lock:
            self._park_and_disable()

    def stop(self) -> None:
        with self._lock:
            self.driver.stop()

    def reset(self) -> None:
        with self._lock:
            self.driver.reset()

    def read_state(self) -> RobotState:
        with self._lock:
            return self.manual_control.read_state()

    def send_joint_action(self, joint_position: list[float], gripper_position: float | None = None) -> ExecutionRecord:
        with self._lock:
            return self.manual_control.send_joint_action(joint_position, gripper_position)

    def move_to_joint_action(self, joint_position: list[float], gripper_position: float | None = None) -> ExecutionRecord:
        with self._lock:
            return self.manual_control.move_to_joint_action(joint_position, gripper_position)

    def set_gripper(self, gripper_position: float) -> ExecutionRecord:
        with self._lock:
            return self.manual_control.set_gripper(gripper_position)

    def home(self) -> ExecutionRecord:
        with self._lock:
            return self.manual_control.home()

    def send_cartesian_pose(self, x_mm: float, y_mm: float, z_mm: float, rx_deg: float, ry_deg: float, rz_deg: float) -> None:
        with self._lock:
            self.manual_control.send_cartesian_pose(x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg)

    def configure_master_slave(
        self,
        linkage_config: int,
        feedback_offset: int = 0x00,
        ctrl_offset: int = 0x00,
        linkage_offset: int = 0x00,
    ) -> None:
        with self._lock:
            self.manual_control.configure_master_slave(linkage_config, feedback_offset, ctrl_offset, linkage_offset)

    def probe_and_configure_sdk_feedback(self) -> dict:
        with self._lock:
            return self.manual_control.probe_and_configure_sdk_feedback()

    def auto_identify_feedback_ids(self) -> dict:
        with self._lock:
            return self.manual_control.auto_identify_feedback_ids()

    def build_controller(self) -> Controller:
        return Controller(
            driver=self.driver,
            policy=self.policy,
            executor=self.executor,
            camera_driver=self.camera_driver,
            prompt=self.factory.policy_config.get("prompt"),
        )

    def _park_and_disable(self) -> None:
        park = self.factory.robot_config.get("shutdown", {}).get("park", {})
        rest_degrees = park.get("joint_position_deg", [-90.0, 0.0, 0.0, 0.0, 40.0, 0.0])
        if len(rest_degrees) != 6 or any(not math.isfinite(float(value)) for value in rest_degrees):
            raise ValueError("shutdown.park.joint_position_deg must contain six finite values")
        target = [math.radians(float(value)) for value in rest_degrees]
        tolerance = math.radians(float(park.get("tolerance_deg", 2.0)))
        stable_samples_required = int(park.get("stable_samples", 3))
        timeout_s = float(park.get("timeout_s", 45.0))
        poll_interval_s = float(park.get("poll_interval_s", 0.05))
        if tolerance < 0.0 or stable_samples_required < 1 or timeout_s <= 0.0 or poll_interval_s < 0.0:
            raise ValueError("shutdown.park timing, stability, and tolerance values are invalid")

        state = self.manual_control.read_state()
        if not state.is_enabled:
            return
        self._validate_park_feedback(state, previous_timestamp=None)
        previous_timestamp = state.timestamp
        started = time.monotonic()
        stable_samples = 0

        while time.monotonic() - started <= timeout_s:
            self.manual_control.send_joint_action(target, state.gripper_position)
            if poll_interval_s:
                time.sleep(poll_interval_s)
            state = self.manual_control.read_state()
            self._validate_park_feedback(state, previous_timestamp)
            previous_timestamp = state.timestamp
            if all(abs(actual - desired) <= tolerance for actual, desired in zip(state.joint_position, target)):
                stable_samples += 1
                if stable_samples >= stable_samples_required:
                    self.driver.disable()
                    return
            else:
                stable_samples = 0
        raise RuntimeError("Park timed out before target stability was confirmed; motors remain enabled")

    def _validate_park_feedback(self, state: RobotState, previous_timestamp: float | None) -> None:
        if not math.isfinite(state.timestamp) or state.timestamp <= 0.0:
            raise RuntimeError("Park feedback timestamp is invalid; motors remain enabled")
        if previous_timestamp is not None and state.timestamp <= previous_timestamp:
            raise RuntimeError("Park feedback did not advance; motors remain enabled")
        if not state.is_enabled:
            raise RuntimeError("Motor enable feedback was lost while parking; disable was not sent")
        if state.error_code not in (None, 0):
            raise RuntimeError(f"Robot fault {state.error_code} while parking; motors remain enabled")
        if any(state.joint_communication_flags):
            raise RuntimeError("Joint communication fault while parking; motors remain enabled")
        if any(state.joint_limit_flags):
            raise RuntimeError("Joint limit fault while parking; motors remain enabled")

    def __enter__(self) -> "RobotSession":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.disconnect()
