from __future__ import annotations

from abc import ABC, abstractmethod

from src.core.action import Action
from src.core.observation import Observation


class BasePolicy(ABC):
    @abstractmethod
    def infer(self, observation: Observation) -> Action:
        raise NotImplementedError
