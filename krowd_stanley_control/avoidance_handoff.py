"""Hand the vehicle to cone_avoidance_1005 inside the map's avoidance zones.

Decided with the user on 2026-10-07:

  approaching a zone   slow (decel a) to approach_speed, held until the brake
                       point; brake when the braking distance of the current
                       speed (v*latency + v^2/2a_brake, the red-light model)
                       reaches the zone start so the front axle stands still
                       AT the start waypoint, not inside the zone (2026-10-09);
  stopped              then HAND OFF: stanley_node
                       publishes /stanley_node/handoff and goes SILENT on the
                       actuator topics (it must not fight cone_avoidance's
                       command_guard on /brake);
  handed off           avoidance_supervisor_node runs the cone_avoidance_1005
                       launch; stanley_node keeps projecting the vehicle so the
                       supervisor knows when the zone end has been passed;
  resume               on the supervisor's /stanley_node/resume (cone mission
                       DONE, zone end passed, or a timeout) tracking continues
                       from wherever the vehicle is; the zone is latched so it
                       is not handed off again until the vehicle has come
                       round to before its start (a loop map).

Zones are the map's avoidance_zones.yaml (localization_monitor_node:
avoidance_zones, waypoint idx ranges) resolved to arc length by
route_publisher_node exactly like the traffic-light zones (signal_stop.py).
The reference line cone_avoidance fixes from the vehicle heading at its start,
so the hand-off waits for the vehicle to stand still aligned with the route
(within heading_tolerance), at most align_timeout_s.

Plain numbers, no ROS.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

from krowd_stanley_control.signal_stop import SignalZone, _index_ranges, _zone_list, brake_distance, zones_on_route

AVOID = "avoid"

DRIVE = "DRIVE"
APPROACH = "APPROACH"
STOPPING = "STOPPING"
HANDED_OFF = "HANDED_OFF"


def read_avoidance_zones(map_dir: Path) -> list[tuple[int, int, str]]:
    """(first idx, last idx, "avoid") of every avoidance zone of a map; [] when
    the map has none. ValueError for a malformed entry."""

    specs = _zone_list(Path(map_dir) / "avoidance_zones.yaml",
                       "localization_monitor_node", "avoidance_zones")
    return [(first, last, AVOID) for first, last in sorted(_index_ranges(specs, "avoidance zone"))]


@dataclass(frozen=True)
class HandoffVerdict:
    state: str
    zone: SignalZone | None
    speed_cap_mps: float | None   # APPROACH: the cap; else None
    brake: bool                   # STOPPING: brake where we are
    past_end: bool = False        # HANDED_OFF: the vehicle has passed the zone end


class AvoidanceHandoff:
    def __init__(
        self,
        approach_speed_mps: float = 5.0 / 3.6,
        approach_decel_mps2: float = 0.5,
        stop_speed_mps: float = 0.3,
        stop_settle_s: float = 1.0,
        heading_tolerance_rad: float = math.radians(5.0),
        align_timeout_s: float = 10.0,
        relatch_margin_m: float = 5.0,
        brake_latency_s: float = 1.0,
        brake_decel_mps2: float = 3.95,
    ):
        self.approach_speed = float(approach_speed_mps)
        self.approach_decel = max(float(approach_decel_mps2), 0.0)
        self.brake_latency_s = max(float(brake_latency_s), 0.0)
        self.brake_decel = max(float(brake_decel_mps2), 0.0)
        self.stop_speed = float(stop_speed_mps)
        self.stop_settle_s = float(stop_settle_s)
        self.heading_tolerance = float(heading_tolerance_rad)
        self.align_timeout_s = float(align_timeout_s)
        self.relatch_margin = float(relatch_margin_m)
        self.zones: list[SignalZone] = []
        self.state = DRIVE
        self.zone: SignalZone | None = None
        self.latched: set[str] = set()
        self._still_since: float | None = None
        self._stopping_since: float | None = None
        self.last_note = ""

    def set_zones(self, zones: list[SignalZone]) -> None:
        self.zones = sorted(zones, key=lambda z: z.start_s)
        if self.zone is not None and self.zone not in self.zones:
            self.reset()

    def reset(self) -> None:
        self.state = DRIVE
        self.zone = None
        self._still_since = None
        self._stopping_since = None

    def resume(self) -> None:
        """The supervisor gives the vehicle back; the zone stays latched."""

        if self.zone is not None:
            self.latched.add(self.zone.label)
        self.reset()

    def step(
        self,
        now_s: float,
        s: float,
        speed_mps: float,
        heading_error_rad: float,
        route_length: float = 0.0,
        closed_loop: bool = False,
    ) -> HandoffVerdict:
        # Unlatch a zone once the vehicle is clearly before its start again.
        for zone in self.zones:
            if zone.label in self.latched and self._distance_ahead(s, zone, route_length, closed_loop) > self.relatch_margin:
                self.latched.discard(zone.label)

        if self.state == HANDED_OFF:
            past = s > self.zone.end_s or (closed_loop and route_length > 0.0 and self._distance_ahead(s, self.zone, route_length, closed_loop) > self.relatch_margin)
            return HandoffVerdict(HANDED_OFF, self.zone, None, False, past)

        if self.state == STOPPING:
            if speed_mps <= self.stop_speed:
                self._still_since = self._still_since if self._still_since is not None else now_s
            else:
                self._still_since = None
            still = self._still_since is not None and now_s - self._still_since >= self.stop_settle_s
            aligned = abs(heading_error_rad) <= self.heading_tolerance
            waited = now_s - self._stopping_since if self._stopping_since is not None else 0.0
            if still and (aligned or waited >= self.align_timeout_s):
                self.last_note = "" if aligned else (
                    f"heading {math.degrees(heading_error_rad):+.1f} deg off the route after "
                    f"{waited:.0f} s - handing off anyway")
                self.state = HANDED_OFF
                return HandoffVerdict(HANDED_OFF, self.zone, None, False)
            return HandoffVerdict(STOPPING, self.zone, None, True)

        # DRIVE / APPROACH: the next unlatched zone ahead.
        nearest, distance = None, math.inf
        for zone in self.zones:
            if zone.label in self.latched:
                continue
            if zone.contains(s):
                nearest, distance = zone, 0.0
                break
            d = self._distance_ahead(s, zone, route_length, closed_loop)
            if 0.0 <= d < distance:
                nearest, distance = zone, d
        if nearest is None:
            self.state = DRIVE
            self.zone = None
            return HandoffVerdict(DRIVE, None, None, False)
        # Brake when the braking distance of the current speed reaches the
        # zone start, or when already inside the zone (started in it, resumed
        # in it): the vehicle then stands still at (not past) the start.
        if distance <= self.brake_distance(speed_mps) or nearest.contains(s):
            self.state = STOPPING
            self.zone = nearest
            self._stopping_since = now_s
            self._still_since = now_s if speed_mps <= self.stop_speed else None
            return HandoffVerdict(STOPPING, nearest, None, True)
        # Decelerate towards the brake point; never below approach_speed so the
        # vehicle keeps rolling until the brake decision.
        cap = None
        if self.approach_decel > 0.0:
            room = max(distance - self.brake_distance(self.approach_speed), 0.0)
            cap = max(self.approach_speed, math.sqrt(2.0 * self.approach_decel * room))
        self.state = APPROACH
        self.zone = nearest
        return HandoffVerdict(APPROACH, nearest, cap, False)

    def brake_distance(self, speed_mps: float) -> float:
        return brake_distance(speed_mps, self.brake_latency_s, self.brake_decel)

    @staticmethod
    def _distance_ahead(s: float, zone: SignalZone, route_length: float, closed_loop: bool) -> float:
        d = zone.start_s - s
        if d < 0.0 and closed_loop and route_length > 0.0:
            d += route_length
        return d


__all__ = [
    "AVOID", "DRIVE", "APPROACH", "STOPPING", "HANDED_OFF",
    "AvoidanceHandoff", "HandoffVerdict", "read_avoidance_zones", "zones_on_route",
]
