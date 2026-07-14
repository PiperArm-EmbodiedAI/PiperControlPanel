from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.drivers.mock_driver import MockDriver
from src.drivers.piper_driver import PiperDriver


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["mock", "real"], default="mock")
    args = parser.parse_args()

    driver = MockDriver() if args.mode == "mock" else PiperDriver()
    driver.connect()
    driver.enable()
    driver.close_gripper()
    print(driver.get_state())


if __name__ == "__main__":
    main()
