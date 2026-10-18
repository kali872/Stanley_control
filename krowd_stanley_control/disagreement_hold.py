"""Stop on a heading disagreement only once it has LASTED.

The ESKF (RTK course) and MOLA map-localization headings are compared every
control cycle. MOLA's pose arrives at 2-3 Hz, so in a bend it lags the ESKF by
up to a few hundred milliseconds - at 25 deg/s that alone is 5-8 deg - and on
top of that the two disagreed by a steady ~7 deg on map 20260923_113433. On the
2026-09-24 drive (stanley_0924_191013) this tripped the 15 deg limit 14 times
for 0.1-0.5 s each, always in bends, and every brake pulse left the vehicle
standing for 3-6 s.

So a disagreement above `limit` must persist for `hold_s` before it stops the
vehicle. A disagreement above `immediate` (a wrong heading, not a lagging one:
the 2026-09-21 case was 124 deg) still stops at once. The hold resets only when
a comparison comes back under the limit; a gap in the comparisons (no MOLA
pose) neither resets nor advances it past what the clock says.

Plain numbers, no ROS.
"""


class DisagreementHold:
    def __init__(self, hold_s: float, immediate_rad: float):
        if hold_s < 0.0 or immediate_rad <= 0.0:
            raise ValueError(
                f"disagreement hold needs hold >= 0 and immediate > 0: {hold_s}, {immediate_rad}"
            )
        self.hold_s = float(hold_s)
        self.immediate_rad = float(immediate_rad)
        self._since_s: float | None = None

    def reset(self) -> None:
        self._since_s = None

    def held_s(self, now_s: float) -> float:
        return 0.0 if self._since_s is None else max(now_s - self._since_s, 0.0)

    def update(self, now_s: float, error_rad: float, limit_rad: float) -> bool:
        """True when the vehicle must stop for this disagreement."""

        error = abs(error_rad)
        if error <= limit_rad:
            self._since_s = None
            return False
        if self._since_s is None:
            self._since_s = now_s
        if error >= self.immediate_rad:
            return True
        return now_s - self._since_s >= self.hold_s
