#!/usr/bin/env python3
"""
Test Suite: Autonomous Mission Trajectory Math & Ramp Curve Simulator
Tests kinematic equations, duration calculations, continuity, and command parsing.
"""

import unittest
import math


def calculate_maneuver_duration(cmd: str, arg: float, speed_cm_per_sec: float = 65.0) -> float:
    cmd = cmd.upper()
    if cmd in ("FORWARD", "BACKWARD", "LEFT", "RIGHT"):
        dist = arg if arg > 0 else 30.0
        return max(0.6, dist / speed_cm_per_sec)
    elif cmd in ("UP", "DOWN"):
        dist = arg if arg > 0 else 30.0
        return max(0.6, dist / (speed_cm_per_sec * 0.8))
    elif cmd in ("YAW", "ROTATE"):
        deg = arg if arg != 0 else 90.0
        return max(0.5, abs(deg) / 120.0)
    elif cmd in ("HOVER", "STAY", "WAIT"):
        return arg if arg > 0 else 2.5
    elif cmd == "TAKEOFF":
        return 3.0
    elif cmd == "LAND":
        return 3.5
    elif cmd == "FLIP":
        return 1.8
    return 0.0


def simulate_ramp_curve(target_val: int, duration_sec: float, ramp_duration: float = 0.15, dt: float = 0.01):
    """
    Simulates time-series axis values over the duration of a maneuver.
    Returns list of (t, val) tuples.
    """
    samples = []
    t = 0.0

    # Ensure effective ramp_duration does not exceed half the total duration
    eff_ramp = min(ramp_duration, duration_sec / 2.0) if duration_sec > 0 else ramp_duration

    while t <= duration_sec + 1e-6:
        if t < eff_ramp:
            factor = t / eff_ramp if eff_ramp > 0 else 1.0
            val = int(round(128 + (target_val - 128) * factor))
        elif t > duration_sec - eff_ramp:
            remaining = duration_sec - t
            factor = max(0.0, remaining / eff_ramp) if eff_ramp > 0 else 0.0
            val = int(round(128 + (target_val - 128) * factor))
        else:
            val = target_val

        val = max(1, min(255, val))
        samples.append((round(t, 4), val))
        t += dt

    return samples


class TestTrajectoryCalculations(unittest.TestCase):

    def test_kinematic_duration_formulas(self):
        """Verify speed-to-time kinematic formulas."""
        # 130 cm forward at 65 cm/s = 2.0s
        self.assertAlmostEqual(calculate_maneuver_duration("FORWARD", 130.0), 2.0, places=3)
        # Small distance should clamp to min duration 0.6s
        self.assertAlmostEqual(calculate_maneuver_duration("FORWARD", 10.0), 0.6, places=3)

        # 52 cm vertical climb at 52 cm/s (65 * 0.8) = 1.0s
        self.assertAlmostEqual(calculate_maneuver_duration("UP", 52.0), 1.0, places=3)

        # 360 degree yaw at 120 deg/s = 3.0s
        self.assertAlmostEqual(calculate_maneuver_duration("YAW", 360.0), 3.0, places=3)
        # Negative yaw (-180 deg) at 120 deg/s = 1.5s
        self.assertAlmostEqual(calculate_maneuver_duration("ROTATE", -180.0), 1.5, places=3)

    def test_ramp_continuity_and_boundaries(self):
        """Verify that ramp curves start and end smoothly at 128 with no spikes."""
        samples = simulate_ramp_curve(target_val=213, duration_sec=1.0, ramp_duration=0.15, dt=0.01)

        # Start value must be neutral 128
        self.assertEqual(samples[0][1], 128, "Ramp must start at neutral 128")

        # Peak must reach target value 213
        peak_vals = [s[1] for s in samples if 0.15 <= s[0] <= 0.85]
        self.assertTrue(all(v == 213 for v in peak_vals), "Plateau must hold target value")

        # End value must return to neutral 128
        self.assertEqual(samples[-1][1], 128, "Ramp must end at neutral 128")

    def test_ramp_monotonicity(self):
        """Ramp-up must be non-decreasing, and ramp-down must be non-increasing."""
        samples = simulate_ramp_curve(target_val=213, duration_sec=1.0, ramp_duration=0.15, dt=0.01)

        # Check ramp-up (0.0 to 0.15)
        ramp_up = [s[1] for s in samples if s[0] <= 0.15]
        for i in range(len(ramp_up) - 1):
            self.assertLessEqual(ramp_up[i], ramp_up[i+1], "Ramp up must be monotonic non-decreasing")

        # Check ramp-down (0.85 to 1.0)
        ramp_down = [s[1] for s in samples if s[0] >= 0.85]
        for i in range(len(ramp_down) - 1):
            self.assertGreaterEqual(ramp_down[i], ramp_down[i+1], "Ramp down must be monotonic non-increasing")

    def test_short_duration_adaptation(self):
        """When maneuver duration is shorter than default ramp window (e.g. 0.2s), ramping adapts gracefully."""
        samples = simulate_ramp_curve(target_val=200, duration_sec=0.2, ramp_duration=0.15, dt=0.01)
        self.assertEqual(samples[0][1], 128)
        self.assertEqual(samples[-1][1], 128)
        # Check no overflow
        for _, val in samples:
            self.assertTrue(1 <= val <= 255)


if __name__ == '__main__':
    unittest.main(verbosity=2)
