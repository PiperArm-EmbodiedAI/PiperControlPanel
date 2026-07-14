from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(slots=True)
class CameraDriver:
    mode: str = "none"
    path: str | None = None
    device_index: int = 0

    def capture(self) -> dict[str, Any | None]:
        if self.mode == "none":
            return {"cam_high": None, "cam_left_wrist": None, "cam_right_wrist": None}
        if self.mode == "image_file":
            payload = Path(self.path).read_bytes() if self.path else None
            return {"cam_high": payload, "cam_left_wrist": None, "cam_right_wrist": None}
        if self.mode == "usb_camera":
            try:
                import cv2
            except ImportError as exc:
                raise RuntimeError("opencv-python is required for usb_camera mode") from exc
            cap = cv2.VideoCapture(self.device_index)
            ok, frame = cap.read()
            cap.release()
            if not ok:
                raise RuntimeError(f"Failed to read frame from camera index {self.device_index}")
            return {"cam_high": frame, "cam_left_wrist": None, "cam_right_wrist": None}
        raise ValueError(f"Unsupported camera mode: {self.mode}")
