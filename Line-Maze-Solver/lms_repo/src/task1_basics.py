"""Task 1 - Basic coding (Python).  Run:  python3 task1_basics.py
Equivalent C++ versions (used on the robot) are in lms_line_follower/lms_core.h and are
unit-tested by tests/test_core.cpp.

Hardware Context:
- Tasks 1-8 target the TB6612FNG dual MOSFET H-bridge motor driver powered by a 2S LiPo (7.4 V nominal).
  Signed motor commands in [-255, +255] map to PWMA/PWMB duty cycle (0..255) and direction pins
  (AIN1/AIN2, BIN1/BIN2 with STBY pulled HIGH).
- Task 9 uses the Appendix A hardware configuration (L298N + 3S 11.1 V LiPo).
"""
import math
from typing import List, Optional, Tuple

WEIGHTS = [-2, -1, 0, 1, 2]


def calibrate_readings(
    raw: List[float],
    min_vals: List[float],
    max_vals: List[float],
    scale: float = 1000.0,
    line_is_high: bool = True,
) -> List[float]:
    """Normalize raw sensor values channel-by-channel using stored min/max calibration bounds.

    raw          : five raw ADC sensor values
    min_vals     : empirical minimum baseline per sensor
    max_vals     : empirical maximum reading per sensor
    scale        : upper normalization bound (default 1000.0)
    line_is_high : True if the line produces a higher ADC reading than the background,
                   False to invert (e.g., white line on black floor if white gives lower voltage)
    returns      : normalized values scaled to (0.0 .. scale)
    """
    if not (len(raw) == len(min_vals) == len(max_vals)):
        raise ValueError("inputs must have identical lengths")
    if scale <= 0.0:
        raise ValueError("scale must be positive")

    calibrated = []
    for v, r_min, r_max in zip(raw, min_vals, max_vals):
        span = float(r_max) - float(r_min)
        if span <= 0.0 or math.isnan(v) or math.isinf(v):
            calibrated.append(0.0)
        else:
            norm = ((float(v) - float(r_min)) / span) * scale
            clamped = max(0.0, min(scale, norm))
            calibrated.append(clamped if line_is_high else (scale - clamped))
    return calibrated


def line_position(
    readings: List[float],
    weights: List[int] = WEIGHTS,
    min_total: float = 1e-6,
) -> Optional[float]:
    """Weighted centroid  sum(w_i * v_i) / sum(v_i).

    readings  : five calibrated, non-negative values (left -> right)
    weights   : sensor position weights [-2, -1, 0, 1, 2]
    min_total : minimum total reflectance required to consider the line present
    returns   : position in sensor units (-2 .. +2), or None when all readings are zero
                (no line under the array). Returning None instead of 0.0 matters: 0.0 would
                mean "perfectly centred" and the robot would drive straight off the line.
    """
    if len(readings) != len(weights):
        raise ValueError("need exactly %d readings" % len(weights))

    v = []
    for x in readings:
        fx = float(x)
        if math.isnan(fx) or math.isinf(fx) or fx < 0.0:
            v.append(0.0)  # defensive clip: spec says non-negative
        else:
            v.append(fx)

    total = sum(v)
    if total <= 0.0 or total < min_total:  # explicit all-zero / lost-line case
        return None
    return sum(w * x for w, x in zip(weights, v)) / total


def clamp_motors(left: int, right: int, max_mag: int) -> Tuple[int, int]:
    """Limit each motor command to [-max_mag, +max_mag] while preserving sign (direction).

    For the TB6612FNG driver (Tasks 1-8, 2S 7.4V LiPo) and L298N (Task 9, Appendix A),
    magnitude sets the 8-bit PWM duty cycle (0..255) and sign selects H-bridge direction.
    """
    m = abs(int(max_mag))

    def _clamp_one(val: int) -> int:
        iv = int(val)
        return max(-m, min(m, iv))

    return _clamp_one(left), _clamp_one(right)


if __name__ == "__main__":
    print("line_position examples")
    for r in (
        [0, 0, 1000, 0, 0],
        [0, 1000, 1000, 0, 0],
        [0, 0, 200, 800, 0],
        [1000, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
    ):
        print("  %-22s -> %s" % (r, line_position(r)))
    print("clamp_motors examples (max=255 for TB6612FNG PWMA/PWMB)")
    for l, r in ((300, -300), (100, -50), (-1000, 255), (0, 0)):
        print("  in (%5d,%5d) -> %s" % (l, r, clamp_motors(l, r, 255)))
    print("calibrate_readings examples")
    mins, maxs = [350, 410, 390, 400, 420], [3200, 3800, 3900, 3700, 3600]
    raw_sample = [350, 3800, 2145, 400, 420]
    calibrated = calibrate_readings(raw_sample, mins, maxs)
    print("  raw %s -> calibrated %s -> pos %s" % (raw_sample, calibrated, line_position(calibrated)))
    # self-checks
    assert line_position([0, 0, 1000, 0, 0]) == 0
    assert line_position([0, 1000, 1000, 0, 0]) == -0.5
    assert abs(line_position([0, 0, 200, 800, 0]) - 0.8) < 1e-9
    assert line_position([0, 0, 0, 0, 0]) is None
    assert clamp_motors(300, -300, 255) == (255, -255)
    assert clamp_motors(100, -50, 255) == (100, -50)
    assert calibrate_readings([350, 3800, 390, 400, 420], mins, maxs) == [0.0, 1000.0, 0.0, 0.0, 0.0]
    print("all Task 1 checks passed")
