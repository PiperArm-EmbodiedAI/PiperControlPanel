from __future__ import annotations

from dataclasses import dataclass

import requests

from src.core.action import Action
from src.core.observation import Observation
from src.policies.base_policy import BasePolicy


@dataclass(slots=True)
class RemoteHTTPPolicy(BasePolicy):
    url: str
    timeout_s: float = 5.0

    def infer(self, observation: Observation) -> Action:
        response = requests.post(self.url, json=observation.to_dict(), timeout=self.timeout_s)
        response.raise_for_status()
        data = response.json()
        return Action(
            joint_position=list(data["joint_position"]),
            gripper_position=float(data["gripper_position"]),
        )
