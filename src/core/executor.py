from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Protocol

from src.core.action import Action
from src.core.robot_state import RobotState
from src.core.safety import SafetyLayer, SafetyResult
from src.utils.logger import get_logger


class SupportsSendAction(Protocol):
    def send_action(self, action: Action) -> None:
        ...


@dataclass(slots=True)
class ExecutionRecord:
    requested_action: Action
    safety_result: SafetyResult
    duration_s: float


class Executor:
    def __init__(self, driver: SupportsSendAction, safety_layer: SafetyLayer, rate_hz: float = 10.0) -> None:
        self.driver = driver
        self.safety_layer = safety_layer
        self.rate_hz = float(rate_hz)
        self.logger = get_logger(__name__)
        self._last_execute_monotonic: float | None = None

    def execute(self, action: Action, current_state: RobotState) -> ExecutionRecord:
        started = time.monotonic()
        safety_result = self.safety_layer.apply(action, current_state)
        if not safety_result.accepted or safety_result.action is None:
            raise ValueError(f"Unsafe action rejected: {safety_result.reasons}")

        if safety_result.reasons:
            self.logger.warning("Safety adjusted action: %s", "; ".join(safety_result.reasons))

        self.driver.send_action(safety_result.action)
        self._enforce_rate()
        duration_s = time.monotonic() - started
        return ExecutionRecord(requested_action=action, safety_result=safety_result, duration_s=duration_s)

    def _enforce_rate(self) -> None:
        if self.rate_hz <= 0:
            return
        now = time.monotonic()
        if self._last_execute_monotonic is None:
            self._last_execute_monotonic = now
            return
        period = 1.0 / self.rate_hz
        elapsed = now - self._last_execute_monotonic
        remaining = period - elapsed
        if remaining > 0:
            time.sleep(remaining)
        self._last_execute_monotonic = time.monotonic()
