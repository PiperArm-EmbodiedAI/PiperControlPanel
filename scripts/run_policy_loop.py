from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.app.session import RobotSession
from src.utils.logger import configure_logging


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["mock", "real"], default=None)
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--robot-config", default="config/robot.yaml")
    parser.add_argument("--policy-config", default="config/policy.yaml")
    parser.add_argument("--demo-config", default="config/demo.yaml")
    args = parser.parse_args()

    session = RobotSession.from_config(
        mode=args.mode,
        robot_config_path=args.robot_config,
        policy_config_path=args.policy_config,
        demo_config_path=args.demo_config,
    )
    configure_logging(session.factory.demo_config.get("log_level", "INFO"))

    try:
        session.connect()
        session.enable()
        controller = session.build_controller()
        steps = controller.run(max_steps=args.max_steps or session.factory.demo_config["max_steps"])
        print(f"Completed {len(steps)} control steps in {session.mode} mode")
        print(session.read_state())
    finally:
        session.disconnect()


if __name__ == "__main__":
    main()
