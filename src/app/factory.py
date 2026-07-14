from __future__ import annotations

from pathlib import Path
from typing import Any

from src.core.executor import Executor
from src.core.safety import SafetyLayer
from src.drivers.camera_driver import CameraDriver
from src.drivers.mock_driver import MockDriver
from src.drivers.piper_driver import PiperDriver
from src.policies.mock_policy import MockPolicy
from src.policies.openpi_ws_policy import OpenPIWebSocketPolicy
from src.policies.remote_http_policy import RemoteHTTPPolicy
from src.utils.config import load_yaml


class AppFactory:
    def __init__(
        self,
        robot_config_path: str | Path = "config/robot.yaml",
        policy_config_path: str | Path = "config/policy.yaml",
        demo_config_path: str | Path = "config/demo.yaml",
    ) -> None:
        self.robot_wrapper = load_yaml(robot_config_path)
        self.policy_wrapper = load_yaml(policy_config_path)
        self.demo_wrapper = load_yaml(demo_config_path)
        self.robot_config = self.robot_wrapper["robot"]
        self.policy_config = self.policy_wrapper["policy"]
        self.demo_config = self.demo_wrapper["demo"]

    def resolve_mode(self, mode: str | None = None) -> str:
        return mode or self.demo_config.get("mode") or self.robot_wrapper.get("mode", "mock")

    def build_driver(self, mode: str | None = None):
        resolved_mode = self.resolve_mode(mode)
        if resolved_mode == "mock":
            return MockDriver(
                initial_joint_position=self.robot_config.get("home_joint_position", [0.0] * 6),
                initial_gripper_position=self.robot_config["gripper"]["open"],
            )
        if resolved_mode == "real":
            return PiperDriver(
                can_interface=self.robot_config["can_interface"],
                auto_enable=self.robot_config["auto_enable"],
                gripper_open_value=self.robot_config["gripper"]["open"],
                gripper_closed_value=self.robot_config["gripper"]["closed"],
                move_speed_rate=self.robot_config.get("move_speed_rate", 30),
            )
        raise ValueError(f"Unsupported mode: {resolved_mode}")

    def build_safety(self) -> SafetyLayer:
        return SafetyLayer(
            joint_min=self.robot_config["joint_limits"]["min"],
            joint_max=self.robot_config["joint_limits"]["max"],
            max_delta=self.robot_config["max_delta"],
            gripper_min=self.robot_config["gripper"]["min"],
            gripper_max=self.robot_config["gripper"]["max"],
        )

    def build_executor(self, driver: Any) -> Executor:
        return Executor(
            driver=driver,
            safety_layer=self.build_safety(),
            rate_hz=self.robot_config["control_rate_hz"],
        )

    def build_camera(self) -> CameraDriver:
        camera_config = self.demo_config["camera"]
        return CameraDriver(
            mode=camera_config["mode"],
            path=camera_config.get("path"),
            device_index=camera_config["device_index"],
        )

    def build_policy(self):
        policy_type = self.policy_config["type"]
        if policy_type == "mock":
            mock = self.policy_config["mock"]
            return MockPolicy(
                mode=mock["mode"],
                amplitude=mock["amplitude"],
                period_s=mock["period_s"],
                fixed_joint_position=mock["fixed_joint_position"],
                fixed_gripper_position=mock["fixed_gripper_position"],
            )
        if policy_type == "openpi_ws":
            websocket = self.policy_config["websocket"]
            return OpenPIWebSocketPolicy(url=websocket["url"], timeout_s=websocket["timeout_s"])
        if policy_type == "remote_http":
            http = self.policy_config["http"]
            return RemoteHTTPPolicy(url=http["url"], timeout_s=http["timeout_s"])
        raise ValueError(f"Unsupported policy type: {policy_type}")
