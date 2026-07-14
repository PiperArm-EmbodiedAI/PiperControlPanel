from __future__ import annotations

import time
from contextlib import contextmanager
from typing import Iterator


@contextmanager
def timed() -> Iterator[callable]:
    started = time.monotonic()

    def elapsed() -> float:
        return time.monotonic() - started

    yield elapsed
