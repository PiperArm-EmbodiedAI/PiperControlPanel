from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from src.core.executor import Executor, ExecutionRecord
from src.core.observation import Observation
from src.core.robot_state import RobotState
from src.utils.logger import get_logger


class SupportsState(Protocol):
    def get_state(self) -> RobotState:
        ...


class SupportsPolicy(Protocol):
    def infer(self, observation: Observation):
        ...


class SupportsCamera(Protocol):
    def capture(self) -> dict[str, object | None]:
        ...


@dataclass(slots=True)
class ControlStep:
    state: RobotState
    observation: Observation
    execution: ExecutionRecord


class Controller:
    def __init__(
        self,
        driver: SupportsState,
        policy: SupportsPolicy,
        executor: Executor,
        camera_driver: SupportsCamera | None = None,
        prompt: str | None = None,
    ) -> None:
        self.driver = driver
        self.policy = policy
        self.executor = executor
        self.camera_driver = camera_driver
        self.prompt = prompt
        self.logger = get_logger(__name__)

    def step(self) -> ControlStep:
        state = self.driver.get_state()
        images = self.camera_driver.capture() if self.camera_driver is not None else {
            "cam_high": None,
            "cam_left_wrist": None,
            "cam_right_wrist": None,
        }
        observation = Observation(state=state, images=images, prompt=self.prompt)
        action = self.policy.infer(observation)
        execution = self.executor.execute(action, state)
        return ControlStep(state=state, observation=observation, execution=execution)

    def run(self, max_steps: int | None = None) -> list[ControlStep]:
        steps: list[ControlStep] = []
        step_count = 0
        while max_steps is None or step_count < max_steps:
            control_step = self.step()
            steps.append(control_step)
            step_count += 1
            self.logger.info("Completed control step %s", step_count)
        return steps
