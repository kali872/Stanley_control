"""Is someone else driving? The vehicle moves although we command a stop.

2026-10-17 (bag stanley_tuning_1017_095741): the driver took over at a red
light and drove the course by hand while stanley kept braking. The route
position still advanced through an avoidance zone, so the hand-off fired and
cone_avoidance_1005 was launched into a manually driven vehicle. While the
vehicle moves against our brake for a while, nothing that starts another
controller may happen. Plain numbers, no ROS.
"""

from __future__ import annotations


class ExternalMotion:
    def __init__(self, speed_mps: float = 1.5, hold_s: float = 3.0):
        self.speed = float(speed_mps)
        self.hold_s = float(hold_s)
        self._since: float | None = None
        self.active = False

    def step(self, now_s: float, brake_commanded: bool | None, speed_mps: float) -> bool:
        """True once the vehicle has moved faster than `speed` for `hold_s`
        while the last command was a stop; False as soon as it stands still
        or we are the ones commanding motion."""

        if brake_commanded and abs(speed_mps) > self.speed:
            self._since = self._since if self._since is not None else now_s
            self.active = now_s - self._since >= self.hold_s
        else:
            self._since = None
            self.active = False
        return self.active


__all__ = ["ExternalMotion"]
