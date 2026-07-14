from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.utils.logger import configure_logging


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["mock", "real"], default="mock")
    parser.add_argument("--robot-config", default="config/robot.yaml")
    parser.add_argument("--policy-config", default="config/policy.yaml")
    parser.add_argument("--demo-config", default="config/demo.yaml")
    args = parser.parse_args()

    try:
        from PySide6.QtWidgets import QApplication
    except ImportError as exc:
        raise RuntimeError("PySide6 is required for the GUI. Install with: pip install -e .[gui]") from exc

    configure_logging("INFO")
    from src.gui.main_window import MainWindow
    from src.gui.view_model import PiperGuiViewModel

    app = QApplication(sys.argv)
    window = MainWindow(
        PiperGuiViewModel(
            mode=args.mode,
            robot_config_path=args.robot_config,
            policy_config_path=args.policy_config,
            demo_config_path=args.demo_config,
        )
    )
    window.resize(960, 640)
    window.show()
    raise SystemExit(app.exec())


if __name__ == "__main__":
    main()
