import time
from collections import deque


class FPSCounter:
    def __init__(self, alpha: float = 0.05):
        self.alpha = alpha
        self.smoothed_fps: float = 0.0
        self._last_time: float = time.perf_counter()

    def tick(self) -> float:
        now = time.perf_counter()
        dt = now - self._last_time
        self._last_time = now
        if dt > 0:
            instant_fps = 1.0 / dt
        else:
            instant_fps = 0.0
        if self.smoothed_fps == 0.0:
            self.smoothed_fps = instant_fps
        else:
            self.smoothed_fps = (
                self.alpha * instant_fps
                + (1.0 - self.alpha) * self.smoothed_fps
            )
        return self.smoothed_fps


class RateTracker:
    def __init__(self, window_size: int = 100):
        self._timestamps: deque = deque(maxlen=window_size)

    def tick(self) -> None:
        self._timestamps.append(time.perf_counter())

    @property
    def rate_hz(self) -> float:
        if len(self._timestamps) < 2:
            return 0.0
        duration = self._timestamps[-1] - self._timestamps[0]
        if duration <= 0:
            return 0.0
        return (len(self._timestamps) - 1) / duration
