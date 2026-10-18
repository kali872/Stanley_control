"""What the simulated vehicle's sensors report, as opposed to what it does.

The bicycle model in bicycle_sim.py is the truth. On the car Stanley never sees
the truth: it sees the ESKF's estimate, a quantised steering encoder, and - until
the ESKF has verified its heading on an RTK course - a heading from a saved
prior. These are the plain-number models the simulator puts between the truth
and Stanley. TF and the RViz footprint stay on the truth, so the gap is
visible.

Numbers measured from stanley_tuning_0922_235531 (/ekf/odometry at 100 Hz,
185 s, moving samples, residual about a 0.5 s moving average):

    position white noise   0.6 cm      yaw white noise        0.14 deg
    yaw-rate white noise   0.58 deg/s  position jumps > 5 cm  none
    ESKF position sigma    18 cm median (its own covariance, an upper bound)
    ESKF vs RTK position   5.4-7.9 cm per axis (three drives, 2026-09-24/26):
                           the slow-drift sigma used since 2026-09-27
    /encoder               50 Hz, 0.01 rad steps, 0.026 rad jitter at a
                           steady command

and from krowd_sensors' gps_imu_eskf.yaml (the ESKF's verification rule):
3 consecutive RTK-fixed course samples above 1 m/s with |yaw rate| <= 0.10
rad/s, no more than 1 s apart; the GPS antenna sits 2.5038 m ahead of base_link.

No ROS in here.
"""

from dataclasses import dataclass
import math
import random


class GaussMarkov:
    """First-order Gauss-Markov drift: stationary sigma, correlation time tau."""

    def __init__(self, sigma: float, tau_s: float, rng: random.Random):
        self.sigma = max(float(sigma), 0.0)
        self.tau_s = float(tau_s)
        self._rng = rng
        self.value = rng.gauss(0.0, self.sigma) if self.sigma > 0.0 else 0.0

    def step(self, dt_s: float) -> float:
        if self.sigma <= 0.0 or dt_s <= 0.0:
            return self.value
        if self.tau_s <= 0.0:
            self.value = self._rng.gauss(0.0, self.sigma)
            return self.value
        a = math.exp(-dt_s / self.tau_s)
        self.value = a * self.value + self._rng.gauss(
            0.0, self.sigma * math.sqrt(1.0 - a * a)
        )
        return self.value


@dataclass(frozen=True)
class NoiseSample:
    dx: float
    dy: float
    dyaw: float
    dyaw_rate: float


class EstimationNoise:
    """Error of the ESKF estimate against the truth: white noise on position,
    yaw and yaw rate, plus a slow position drift. All zero = a perfect ESKF."""

    def __init__(
        self,
        position_sigma_m: float = 0.0,
        yaw_sigma_rad: float = 0.0,
        yaw_rate_sigma_rad_s: float = 0.0,
        drift_sigma_m: float = 0.0,
        drift_tau_s: float = 10.0,
        seed: int = 0,
    ):
        self._rng = random.Random(seed)
        self.position_sigma = max(float(position_sigma_m), 0.0)
        self.yaw_sigma = max(float(yaw_sigma_rad), 0.0)
        self.yaw_rate_sigma = max(float(yaw_rate_sigma_rad_s), 0.0)
        self._drift_x = GaussMarkov(drift_sigma_m, drift_tau_s, self._rng)
        self._drift_y = GaussMarkov(drift_sigma_m, drift_tau_s, self._rng)

    def _white(self, sigma: float) -> float:
        return self._rng.gauss(0.0, sigma) if sigma > 0.0 else 0.0

    def sample(self, dt_s: float) -> NoiseSample:
        return NoiseSample(
            dx=self._drift_x.step(dt_s) + self._white(self.position_sigma),
            dy=self._drift_y.step(dt_s) + self._white(self.position_sigma),
            dyaw=self._white(self.yaw_sigma),
            dyaw_rate=self._white(self.yaw_rate_sigma),
        )


class SimulatedEncoder:
    """The steering encoder: the actual command-side angle, quantised, with the
    jitter measured at a steady command."""

    def __init__(self, resolution_rad: float = 0.0, noise_rad: float = 0.0, seed: int = 1):
        self.resolution = max(float(resolution_rad), 0.0)
        self.noise = max(float(noise_rad), 0.0)
        self._rng = random.Random(seed)

    def read(self, command_side_rad: float) -> float:
        value = command_side_rad
        if self.noise > 0.0:
            value += self._rng.gauss(0.0, self.noise)
        if self.resolution > 0.0:
            value = round(value / self.resolution) * self.resolution
        return value


class RtkCourseVerifier:
    """When the ESKF would set /ekf/heading_verified from RTK course alone.

    Called once per GPS fix (gps_rate_hz). A fix counts when the vehicle is
    above min_speed with |yaw rate| <= max_yaw_rate; `samples` consecutive
    counting fixes, each within max_gap of the previous, verify. RTK is taken
    as fixed and the course as consistent: the simulated course is the true
    heading, so the ESKF's 5 deg spread check always passes. Once verified it
    stays verified (the topic is latched)."""

    def __init__(
        self,
        samples: int = 3,
        min_speed_mps: float = 1.0,
        max_yaw_rate_rad_s: float = 0.10,
        max_gap_s: float = 1.0,
    ):
        self.samples = max(int(samples), 1)
        self.min_speed = float(min_speed_mps)
        self.max_yaw_rate = float(max_yaw_rate_rad_s)
        self.max_gap = float(max_gap_s)
        self.verified = False
        self._count = 0
        self._last_s = None

    def update(self, now_s: float, speed_mps: float, yaw_rate_rad_s: float) -> bool:
        if self.verified:
            return True
        good = speed_mps >= self.min_speed and abs(yaw_rate_rad_s) <= self.max_yaw_rate
        if not good:
            self._count = 0
            self._last_s = None
            return False
        if self._last_s is not None and now_s - self._last_s > self.max_gap:
            self._count = 0
        self._count += 1
        self._last_s = now_s
        if self._count >= self.samples:
            self.verified = True
        return self.verified


def prior_heading_pose(
    x: float, y: float, yaw: float, heading_error_rad: float, lever_arm_m: float
) -> tuple[float, float, float]:
    """The pose an ESKF reports whose heading is off by heading_error_rad while
    its GPS antenna position (lever_arm_m ahead of base_link) is right: it puts
    base_link one lever arm behind the antenna along the WRONG heading."""

    antenna_x = x + lever_arm_m * math.cos(yaw)
    antenna_y = y + lever_arm_m * math.sin(yaw)
    wrong = yaw + heading_error_rad
    return (
        antenna_x - lever_arm_m * math.cos(wrong),
        antenna_y - lever_arm_m * math.sin(wrong),
        math.atan2(math.sin(wrong), math.cos(wrong)),
    )
