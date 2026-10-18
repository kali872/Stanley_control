"""Right-turn pause: stop at the stop line of a pause zone, hold, then go.

Decided with the user on 2026-10-17: before a right turn the vehicle must come
to a full stop for pause_hold_sec (3 s) and then drive on. Treated like a
traffic-light zone whose light is red until the hold has elapsed:

  approaching a zone   slow (decel a) to zone_speed by its start;
  inside the zone      roll at zone_speed up to the stop point: the detected
                       stop line (StoplineEstimator, stop_buffer_m before it)
                       or, without one, the zone END waypoint - whichever comes
                       first - braking one braking distance (the red-light
                       model v*latency + v^2/2a) before it;
  standing             speed <= stop_speed for hold_s -> released;
  released             the zone is latched until the vehicle is clearly before
                       its start again (a loop map), so it does not stop twice.

Zones are the map's pause_zones.yaml (stanley_node: pause_zones, waypoint idx
ranges; the LAST waypoint is the stop point) resolved to arc length by
route_publisher_node exactly like the traffic-light zones (signal_stop.py).
Plain numbers, no ROS.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

from krowd_stanley_control.signal_stop import SignalZone, _index_ranges, _zone_list, brake_distance

PAUSE = "pause"

DRIVE = "DRIVE"
APPROACH = "APPROACH"
ROLLING = "ROLLING"     # inside the zone, rolling up to the stop point
HOLDING = "HOLDING"     # braking / standing for the hold


def read_pause_zones(map_dir: Path) -> list[tuple[int, int, str]]:
    """(first idx, last idx, "pause") of every pause zone of a map; [] when the
    map has none. ValueError for a malformed entry."""

    specs = _zone_list(Path(map_dir) / "pause_zones.yaml", "stanley_node", "pause_zones")
    return [(first, last, PAUSE) for first, last in sorted(_index_ranges(specs, "pause zone"))]


@dataclass(frozen=True)
class PauseVerdict:
    state: str
    zone: SignalZone | None
    speed_cap_mps: float | None   # APPROACH / ROLLING: the cap; else None
    brake: bool                   # HOLDING: brake where we are
    held_s: float = 0.0           # HOLDING: time standing still so far
    room_m: float | None = None   # ROLLING: front bumper to the stop point
    reason: str = ""
    released: bool = False        # this cycle ended the hold


class PauseStop:
    def __init__(
        self,
        zone_speed_mps: float = 7.0 / 3.6,
        approach_decel_mps2: float = 0.5,
        stop_speed_mps: float = 0.3,
        hold_s: float = 3.0,
        relatch_margin_m: float = 5.0,
        brake_latency_s: float = 1.0,
        brake_decel_mps2: float = 3.95,
    ):
        if not zone_speed_mps > 0.0:
            raise ValueError(f"zone speed must be > 0, got {zone_speed_mps}")
        self.zone_speed = float(zone_speed_mps)
        self.approach_decel = max(float(approach_decel_mps2), 0.0)
        self.stop_speed = float(stop_speed_mps)
        self.hold_s = max(float(hold_s), 0.0)
        self.relatch_margin = float(relatch_margin_m)
        self.brake_latency_s = max(float(brake_latency_s), 0.0)
        self.brake_decel = max(float(brake_decel_mps2), 0.0)
        self.zones: list[SignalZone] = []
        self.state = DRIVE
        self.zone: SignalZone | None = None
        self.latched: set[str] = set()
        self._still_since: float | None = None
        self._brake_reason = ""

    def set_zones(self, zones: list[SignalZone]) -> None:
        self.zones = sorted(zones, key=lambda z: z.start_s)
        if self.zone is not None and self.zone not in self.zones:
            self.reset()

    def reset(self) -> None:
        self.state = DRIVE
        self.zone = None
        self._still_since = None
        self._brake_reason = ""

    def brake_distance(self, speed_mps: float) -> float:
        return brake_distance(speed_mps, self.brake_latency_s, self.brake_decel)

    def stop_decision(
        self,
        zone: SignalZone,
        speed_mps: float,
        s: float,
        line_distance_m: float | None,
        base_to_bumper_m: float,
        front_axle_to_bumper_m: float,
        stop_buffer_m: float,
    ) -> tuple[bool, float, str]:
        """(brake now?, room from the front bumper to the stop point, which
        point). The stop point is the stop line (stop_buffer_m before it) or
        the zone end, whichever is nearer; `s` is the front axle's arc position."""

        room_zone = zone.end_s - (s + front_axle_to_bumper_m)
        room, which = room_zone, "zone end"
        if line_distance_m is not None:
            room_line = line_distance_m - base_to_bumper_m - stop_buffer_m
            if room_line < room_zone:
                room, which = room_line, "stop line"
        return room <= self.brake_distance(speed_mps), room, which

    def step(
        self,
        now_s: float,
        s: float,
        speed_mps: float,
        line_distance_m: float | None,
        base_to_bumper_m: float,
        front_axle_to_bumper_m: float,
        stop_buffer_m: float,
        route_length: float = 0.0,
        closed_loop: bool = False,
    ) -> PauseVerdict:
        # Unlatch a zone once the vehicle is clearly before its start again.
        for zone in self.zones:
            if zone.label in self.latched and self._distance_ahead(s, zone, route_length, closed_loop) > self.relatch_margin:
                self.latched.discard(zone.label)

        if self.state == HOLDING:
            if speed_mps <= self.stop_speed:
                self._still_since = self._still_since if self._still_since is not None else now_s
            else:
                self._still_since = None
            held = now_s - self._still_since if self._still_since is not None else 0.0
            if held >= self.hold_s:
                zone = self.zone
                self.latched.add(zone.label)
                self.reset()
                return PauseVerdict(DRIVE, zone, None, False, held, None, "hold complete", True)
            return PauseVerdict(HOLDING, self.zone, None, True, held, None, self._brake_reason)

        inside = next((z for z in self.zones if z.label not in self.latched and z.contains(s)), None)
        if inside is not None:
            brake, room, which = self.stop_decision(
                inside, speed_mps, s, line_distance_m,
                base_to_bumper_m, front_axle_to_bumper_m, stop_buffer_m,
            )
            if brake:
                self.state = HOLDING
                self.zone = inside
                self._still_since = now_s if speed_mps <= self.stop_speed else None
                self._brake_reason = f"at the {which}"
                return PauseVerdict(HOLDING, inside, None, True, 0.0, room, self._brake_reason)
            self.state = ROLLING
            self.zone = inside
            return PauseVerdict(ROLLING, inside, self.zone_speed, False, 0.0, room, f"rolling up to the {which}")

        # Approaching the next unlatched zone: be at the zone speed when it starts.
        nearest, distance = None, math.inf
        for zone in self.zones:
            if zone.label in self.latched:
                continue
            d = self._distance_ahead(s, zone, route_length, closed_loop)
            if 0.0 <= d < distance:
                nearest, distance = zone, d
        if nearest is None or self.approach_decel <= 0.0:
            self.state = DRIVE
            self.zone = None
            return PauseVerdict(DRIVE, None, None, False)
        cap = math.sqrt(self.zone_speed ** 2 + 2.0 * self.approach_decel * distance)
        self.state = APPROACH
        self.zone = nearest
        return PauseVerdict(APPROACH, nearest, cap, False)

    @staticmethod
    def _distance_ahead(s: float, zone: SignalZone, route_length: float, closed_loop: bool) -> float:
        d = zone.start_s - s
        if d < 0.0 and closed_loop and route_length > 0.0:
            d += route_length
        return d


__all__ = ["PAUSE", "DRIVE", "APPROACH", "ROLLING", "HOLDING", "PauseStop", "PauseVerdict", "read_pause_zones"]
