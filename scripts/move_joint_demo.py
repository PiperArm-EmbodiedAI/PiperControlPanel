from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.core.action import Action
from src.core.executor import Executor
from src.core.safety import SafetyLayer
from src.drivers.mock_driver import MockDriver
from src.drivers.piper_driver import PiperDriver
from src.utils.config import load_yaml


def build_safety() -> SafetyLayer:
    robot_config = load_yaml("config/robot.yaml")["robot"]
    return SafetyLayer(
        joint_min=robot_config["joint_limits"]["min"],
        joint_max=robot_config["joint_limits"]["max"],
        max_delta=robot_config["max_delta"],
        gripper_min=robot_config["gripper"]["min"],
        gripper_max=robot_config["gripper"]["max"],
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["mock", "real"], default="mock")
    parser.add_argument("--joint", nargs=6, type=float, required=True)
    parser.add_argument("--gripper", type=float, default=1.0)
    args = parser.parse_args()

    driver = MockDriver() if args.mode == "mock" else PiperDriver()
    driver.connect()
    driver.enable()

    executor = Executor(driver=driver, safety_layer=build_safety(), rate_hz=10.0)
    state = driver.get_state()
    record = executor.execute(Action(joint_position=args.joint, gripper_position=args.gripper), state)
    print(record)
    print(driver.get_state())


if __name__ == "__main__":
    main()
