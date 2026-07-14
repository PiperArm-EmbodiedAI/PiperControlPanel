from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.drivers.camera_driver import CameraDriver


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["none", "image_file", "usb_camera"], default="none")
    parser.add_argument("--path", default=None)
    parser.add_argument("--device-index", type=int, default=0)
    args = parser.parse_args()

    driver = CameraDriver(mode=args.mode, path=args.path, device_index=args.device_index)
    images = driver.capture()
    print({key: None if value is None else type(value).__name__ for key, value in images.items()})


if __name__ == "__main__":
    main()
