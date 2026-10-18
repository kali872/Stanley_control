"""Stop for a red traffic light inside the map's traffic-light zones.

The rule is krowd_mission_manager's (signal_state.py / mission_logic.py), which
the user chose for Stanley on 2026-10-02 over krowd_final_control's permit
scheme. Re-implemented here rather than imported so that this package does not
depend on, or change, the other one:

  in a traffic-light zone   speed capped at the zone speed (7 km/h);
  RED for the zone's        `stop_frames` consecutive detector frames that say
  direction                 stop, the latest no older than the signal timeout
                            -> brake, speed 0, steering held;
  released                  only by the detector's CONFIRMED go for the same
                            direction, from a frame no older than go_ttl.
                            Latched: braking may carry the vehicle past the end
                            of the zone, and that must not release it on red;
  unknown / off / no        neither stops nor releases. A red outside a zone
  detector                  does nothing.

One addition: the speed cap is approached, not stepped into. Before a zone the
cap is sqrt(zone_speed^2 + 2 a d) for the distance d left to the zone, so the
vehicle (whose drive only coasts down) is at the zone speed when it enters.

Zones are the map's own files next to waypoints.csv, the ones
krowd_mission_manager and localization_monitor_node read:
  traffic_detect_zones.yaml   localization_monitor_node: traffic_detect_zones
  left_turn_zones.yaml        mission_manager_node: left_turn_zones (optional)
Their entries are waypoints.csv idx ranges ("64-68"); they are resolved to arc
length along the published route, which may have dropped or re-ordered
waypoints (backtracking filter, loop closure).

Plain numbers and files, no ROS.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path

import yaml

STRAIGHT = "straight"
LEFT = "left"
DIRECTIONS = (STRAIGHT, LEFT)


# ------------------------------------------------------------ detector frames
@dataclass(frozen=True)
class SignalFrame:
    stamp_s: float
    state: str
    go_straight: bool
    go_left: bool
    go_straight_confirmed: bool
    go_left_confirmed: bool


def parse_frame(text: str) -> SignalFrame:
    """One /traffic_light/detections_json message; ValueError for anything else."""

    try:
        payload = json.loads(text)
        stamp = payload["stamp"]
        return SignalFrame(
            stamp_s=float(stamp["sec"]) + float(stamp["nanosec"]) * 1e-9,
            state=str(payload["state"]),
            go_straight=bool(payload["go_straight"]),
            go_left=bool(payload["go_left"]),
            go_straight_confirmed=bool(payload["go_straight_confirmed"]),
            go_left_confirmed=bool(payload["go_left_confirmed"]),
        )
    except (ValueError, KeyError, TypeError) as error:
        raise ValueError(f"not a traffic-light frame: {error}") from None


def says_stop(frame: SignalFrame, direction: str) -> bool:
    """straight: red (yellow is reported as red), or a green that go_straight
    refuses (3-lamp red + left arrow reads `green`). left: any lit light
    without the left arrow."""

    if direction == LEFT:
        return frame.state in ("red", "green") and not frame.go_left
    return frame.state == "red" or (frame.state == "green" and not frame.go_straight)


def says_go(frame: SignalFrame, direction: str) -> bool:
    return frame.go_left_confirmed if direction == LEFT else frame.go_straight_confirmed


def says_go_hint(frame: SignalFrame, direction: str) -> bool:
    """The unconfirmed go reading of one frame (the detector's go_straight /
    go_left flags)."""

    return frame.go_left if direction == LEFT else frame.go_straight


class SignalTracker:
    """The latest detector frame, when it arrived, and the stop run per direction."""

    def __init__(self, stop_frames: int = 2):
        self.stop_frames = max(1, int(stop_frames))
        self.latest: SignalFrame | None = None
        self.received_s: float | None = None
        self._stop_run = {STRAIGHT: 0, LEFT: 0}
        self._last_stop_s: dict[str, float | None] = {STRAIGHT: None, LEFT: None}
        self._go_seen_s: dict[str, list[float]] = {STRAIGHT: [], LEFT: []}

    def update(self, frame: SignalFrame, received_s: float) -> None:
        self.latest = frame
        self.received_s = received_s
        for direction in self._stop_run:
            if says_stop(frame, direction):
                self._stop_run[direction] += 1
                self._last_stop_s[direction] = received_s
                self._go_seen_s[direction].clear()
            else:
                self._stop_run[direction] = 0
            if says_go_hint(frame, direction):
                seen = self._go_seen_s[direction]
                seen.append(received_s)
                del seen[:-50]

    def go_seen(self, direction: str, now_s: float, window_s: float, frames: int) -> bool:
        """The detector has read a go for `direction` on at least `frames`
        frames within the last `window_s` (not necessarily consecutive, no
        confirmation needed). A stop frame in between clears the count."""

        if frames <= 0:
            return False
        recent = [s for s in self._go_seen_s[direction] if now_s - s <= window_s]
        return len(recent) >= frames

    def since_stop_s(self, direction: str, now_s: float) -> float | None:
        """Seconds since the last frame that said stop for `direction`; None
        when none has."""

        last = self._last_stop_s[direction]
        return None if last is None else now_s - last

    def age_s(self, now_s: float) -> float | None:
        return None if self.received_s is None else now_s - self.received_s

    def red(self, direction: str, now_s: float, timeout_s: float) -> bool:
        age = self.age_s(now_s)
        return (
            age is not None
            and age <= timeout_s
            and self._stop_run[direction] >= self.stop_frames
        )

    def go(self, direction: str, now_s: float, ttl_s: float) -> bool:
        age = self.age_s(now_s)
        return (
            age is not None
            and age <= ttl_s
            and self.latest is not None
            and says_go(self.latest, direction)
        )

    def stop_run(self, direction: str) -> int:
        return self._stop_run[direction]


# ---------------------------------------------------------------------- zones
@dataclass(frozen=True)
class SignalZone:
    """A traffic-light zone as arc length (m) along the published route."""

    label: str        # "64-68", the waypoints.csv idx range
    direction: str    # STRAIGHT | LEFT
    start_s: float
    end_s: float

    def contains(self, s: float) -> bool:
        return self.start_s <= s <= self.end_s


def _index_ranges(specs, what: str) -> list[tuple[int, int]]:
    ranges = []
    for spec in specs or []:
        first_text, separator, last_text = str(spec).strip().partition("-")
        try:
            if not separator:
                raise ValueError
            first, last = int(first_text), int(last_text)
        except ValueError:
            raise ValueError(
                f"{what} '{spec}' is not of the form '<first idx>-<last idx>'"
            ) from None
        if first < 0 or last < first:
            raise ValueError(f"{what} '{spec}': need 0 <= first <= last")
        ranges.append((first, last))
    return ranges


def _zone_list(path: Path, node: str, parameter: str) -> list:
    if not path.is_file():
        return []
    with open(path) as stream:
        document = yaml.safe_load(stream) or {}
    return list(((document.get(node) or {}).get("ros__parameters") or {}).get(parameter) or [])


def read_map_zones(map_dir: Path) -> list[tuple[int, int, str]]:
    """(first idx, last idx, direction) of every traffic-light zone of a map.
    A map without the files, or with empty lists, has no zones. ValueError for
    a malformed entry or a left-turn zone that is not a traffic-light zone."""

    map_dir = Path(map_dir)
    traffic = _index_ranges(
        _zone_list(map_dir / "traffic_detect_zones.yaml",
                   "localization_monitor_node", "traffic_detect_zones"),
        "traffic-detect zone",
    )
    left = _index_ranges(
        _zone_list(map_dir / "left_turn_zones.yaml",
                   "mission_manager_node", "left_turn_zones"),
        "left-turn zone",
    )
    unknown = [zone for zone in left if zone not in traffic]
    if unknown:
        raise ValueError(
            f"left-turn zones {unknown} are not traffic-detect zones {traffic}"
        )
    return [(first, last, LEFT if (first, last) in left else STRAIGHT)
            for first, last in sorted(traffic)]


def _arc_position(route: list[tuple[float, float]], x: float, y: float) -> tuple[float, float]:
    """(arc length along `route` of the point nearest (x, y), its distance)."""

    best = (math.inf, 0.0)
    travelled = 0.0
    for (x0, y0), (x1, y1) in zip(route, route[1:]):
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy)
        t = 0.0 if length <= 0.0 else min(max(((x - x0) * dx + (y - y0) * dy) / length ** 2, 0.0), 1.0)
        distance = math.hypot(x0 + t * dx - x, y0 + t * dy - y)
        if distance < best[0]:
            best = (distance, travelled + t * length)
        travelled += length
    return best[1], best[0]


def zones_on_route(
    specs: list[tuple[int, int, str]],
    waypoints: list[tuple[float, float]],
    route: list[tuple[float, float]],
    max_offset_m: float = 1.0,
) -> tuple[list[SignalZone], list[str]]:
    """Zones resolved onto the published route, and why any were left out.

    `waypoints` are waypoints.csv's points in idx order; `route` is what is
    published (same frame). A zone is kept when both of its end waypoints lie
    within max_offset_m of the route and its start comes before its end.
    """

    zones, problems = [], []
    for first, last, direction in specs:
        label = f"{first}-{last}"
        if last >= len(waypoints):
            problems.append(f"zone {label}: the route has only {len(waypoints)} waypoints")
            continue
        start_s, start_off = _arc_position(route, *waypoints[first])
        end_s, end_off = _arc_position(route, *waypoints[last])
        if max(start_off, end_off) > max_offset_m:
            problems.append(
                f"zone {label}: its waypoints are {max(start_off, end_off):.1f} m off "
                "the published route (dropped or outside the loop)"
            )
            continue
        if end_s < start_s:
            problems.append(f"zone {label}: runs backwards on the published route")
            continue
        zones.append(SignalZone(label, direction, start_s, end_s))
    return zones, problems


def zones_to_json(trajectory_id: str, zones: list[SignalZone]) -> str:
    return json.dumps({
        "trajectory_id": trajectory_id,
        "zones": [
            {"label": z.label, "direction": z.direction,
             "start_s": round(z.start_s, 3), "end_s": round(z.end_s, 3)}
            for z in zones
        ],
    })


def zones_from_json(text: str) -> tuple[str, list[SignalZone]]:
    """(trajectory id, zones); ValueError for anything malformed."""

    try:
        payload = json.loads(text)
        zones = [
            SignalZone(str(z["label"]), str(z["direction"]), float(z["start_s"]), float(z["end_s"]))
            for z in payload["zones"]
        ]
        if any(z.direction not in DIRECTIONS + ("avoid", "pause") for z in zones):
            raise ValueError("unknown direction")
        return str(payload["trajectory_id"]), zones
    except (ValueError, KeyError, TypeError) as error:
        raise ValueError(f"not a signal-zone list: {error}") from None


# ----------------------------------------------------------------- the rule
@dataclass(frozen=True)
class SignalVerdict:
    stop: bool                    # brake for a red
    zone: SignalZone | None       # the zone stopped at, else the zone the vehicle is in
    speed_cap_mps: float | None   # None = no cap from the signal rule
    approaching: SignalZone | None = None


class SignalStop:
    def __init__(self, zone_speed_mps: float, approach_decel_mps2: float = 0.5):
        if not zone_speed_mps > 0.0:
            raise ValueError(f"zone speed must be > 0, got {zone_speed_mps}")
        self.zone_speed = float(zone_speed_mps)
        self.approach_decel = max(float(approach_decel_mps2), 0.0)
        self.zones: list[SignalZone] = []
        self.stopped_at: SignalZone | None = None
        self.last_release = ""   # "go" | "lost" - why the last red stop ended

    def set_zones(self, zones: list[SignalZone]) -> None:
        self.zones = sorted(zones, key=lambda z: z.start_s)
        if self.stopped_at is not None and self.stopped_at not in self.zones:
            self.stopped_at = None

    def reset(self) -> None:
        self.stopped_at = None

    def step(self, s: float, red, go, route_length: float = 0.0,
             closed_loop: bool = False, lost=None) -> SignalVerdict:
        """`s`: arc length of the vehicle along the route. `red(direction)` /
        `go(direction)`: SignalTracker.red / .go for that direction.
        `lost(direction)` (optional): True when the light has not been seen
        saying stop for long enough that the stop is released without a
        confirmed go (2026-10-17, the user: 30 s; the detector lost the light
        once the vehicle stood at the line and the vehicle never moved)."""

        if self.stopped_at is not None:
            if go(self.stopped_at.direction):
                self.last_release = "go"
                self.stopped_at = None
            elif lost is not None and lost(self.stopped_at.direction):
                self.last_release = "lost"
                self.stopped_at = None
            else:
                return SignalVerdict(True, self.stopped_at, 0.0)

        inside = next((z for z in self.zones if z.contains(s)), None)
        if inside is not None:
            if red(inside.direction):
                self.stopped_at = inside
                return SignalVerdict(True, inside, 0.0)
            return SignalVerdict(False, inside, self.zone_speed)

        # Approaching the next zone: be at the zone speed when it starts.
        nearest, distance = None, math.inf
        for zone in self.zones:
            d = zone.start_s - s
            if d < 0.0 and closed_loop and route_length > 0.0:
                d += route_length
            if 0.0 <= d < distance:
                nearest, distance = zone, d
        if nearest is None or self.approach_decel <= 0.0:
            return SignalVerdict(False, None, None)
        cap = math.sqrt(self.zone_speed ** 2 + 2.0 * self.approach_decel * distance)
        return SignalVerdict(False, None, cap, nearest)


# ------------------------------------------------- where to stop for the red
def brake_distance(speed_mps: float, latency_s: float, decel_mps2: float) -> float:
    """Distance covered from deciding to brake until standstill."""

    v = max(speed_mps, 0.0)
    return v * max(latency_s, 0.0) + (v * v / (2.0 * decel_mps2) if decel_mps2 > 0.0 else 0.0)


@dataclass(frozen=True)
class RedStopDecision:
    brake: bool
    reason: str
    room_m: float | None   # front bumper to the stop point; None without a stop line


def red_stop_decision(
    speed_mps: float,
    route_s: float,
    zone_end_s: float,
    line_distance_m: float | None,
    base_to_bumper_m: float,
    front_axle_to_bumper_m: float,
    stop_buffer_m: float,
    latency_s: float,
    decel_mps2: float,
    stop_at_zone_end: bool = False,
) -> RedStopDecision:
    """Brake now, or keep rolling up to the stop line? (the light is red)

    Without a live stop line the vehicle brakes where it is - the rule as it
    was - unless `stop_at_zone_end`: then the zone END waypoint is the stop
    point (2026-10-17, the user: the map's zone ends sit on the stop lines),
    exactly like the right-turn pause. With a line (`line_distance_m`:
    base_link to the line, StoplineEstimator) it keeps the zone speed until
    the front bumper is one braking distance from `stop_buffer_m` before the
    line - and never carries a red past the end of the zone, whatever the line
    detector says (a crosswalk beyond the junction must not pull the vehicle
    through). `route_s` is the front axle's position.
    """

    need = brake_distance(speed_mps, latency_s, decel_mps2)
    if line_distance_m is None and not stop_at_zone_end:
        return RedStopDecision(True, "no stop line: stopping here", None)
    room_zone = zone_end_s - (route_s + front_axle_to_bumper_m)
    room, limited = room_zone, "zone end"
    if line_distance_m is not None:
        room_line = line_distance_m - base_to_bumper_m - stop_buffer_m
        if room_line <= room_zone:
            room, limited = room_line, "stop line"
    if room <= need:
        return RedStopDecision(True, f"at the {limited}", room)
    return RedStopDecision(False, f"rolling up to the {limited}", room)
