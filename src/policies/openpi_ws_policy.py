from __future__ import annotations

import asyncio
import json
from dataclasses import dataclass

import websockets

from src.core.action import Action
from src.core.observation import Observation
from src.policies.base_policy import BasePolicy


@dataclass(slots=True)
class OpenPIWebSocketPolicy(BasePolicy):
    url: str
    timeout_s: float = 5.0

    def infer(self, observation: Observation) -> Action:
        return asyncio.run(self._infer_async(observation))

    async def _infer_async(self, observation: Observation) -> Action:
        payload = {
            "observation": observation.to_dict(),
            "type": "infer",
        }
        async with websockets.connect(self.url, open_timeout=self.timeout_s) as websocket:
            await websocket.send(json.dumps(payload))
            response = await asyncio.wait_for(websocket.recv(), timeout=self.timeout_s)
        data = json.loads(response)
        return Action(
            joint_position=list(data["joint_position"]),
            gripper_position=float(data["gripper_position"]),
        )
