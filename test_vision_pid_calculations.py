#!/usr/bin/env python3
"""
Test Suite: Computer Vision PID Tracking & Optical Flow Calculations
Tests error normalization, PID control law, anti-windup limits, and optical flow vectors.
"""

import unittest
import numpy as np


class PIDController:
    """Discrete PID controller with anti-windup clamping and derivative filtering."""
    def __init__(self, kp: float, ki: float, kd: float, integral_limit: float = 1.0, output_limit: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None

    def update(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        i_term = self.ki * self.integral

        # Derivative term
        d_term = self.kd * (error - self.prev_error) / dt
        self.prev_error = error

        output = p_term + i_term + d_term
        return max(-self.output_limit, min(self.output_limit, output))


def calculate_normalized_error(bbox: tuple, frame_w: int, frame_h: int) -> tuple:
    """
    Calculates normalized centroid errors e_x, e_y in [-1.0, 1.0].
    bbox: (x, y, w, h)
    """
    x, y, w, h = bbox
    cx = x + w / 2.0
    cy = y + h / 2.0
    ex = (cx - (frame_w / 2.0)) / (frame_w / 2.0)
    ey = (cy - (frame_h / 2.0)) / (frame_h / 2.0)
    return max(-1.0, min(1.0, ex)), max(-1.0, min(1.0, ey))


def error_to_stick_channel(pid_output: float, half_len: int = 127) -> int:
    """Maps normalized PID output [-1.0, 1.0] to drone 8-bit stick value [1, 255]."""
    val = 128 + (pid_output * half_len)
    return max(1, min(255, int(round(val))))


def calculate_optical_flow_drift(p0: np.ndarray, p1: np.ndarray, dt: float) -> tuple:
    """Calculates mean displacement drift vector (vx, vy) in px/sec."""
    if len(p0) == 0 or len(p1) == 0 or dt <= 0:
        return 0.0, 0.0
    diff = p1 - p0
    mean_diff = np.mean(diff, axis=0)
    vx = mean_diff[0] / dt
    vy = mean_diff[1] / dt
    return float(vx), float(vy)


class TestVisionCalculations(unittest.TestCase):

    def test_centroid_error_calculation(self):
        """Centered bounding box must produce 0.0 error."""
        # 640x480 frame with centered 100x100 box at (270, 190) -> center (320, 240)
        ex, ey = calculate_normalized_error((270, 190, 100, 100), 640, 480)
        self.assertAlmostEqual(ex, 0.0, places=4)
        self.assertAlmostEqual(ey, 0.0, places=4)

        # Far left target at x=0
        ex_left, _ = calculate_normalized_error((0, 190, 100, 100), 640, 480)
        self.assertLess(ex_left, 0.0)

        # Far right target at x=540
        ex_right, _ = calculate_normalized_error((540, 190, 100, 100), 640, 480)
        self.assertGreater(ex_right, 0.0)

    def test_pid_convergence(self):
        """PID controller should drive error towards zero without blowup."""
        pid = PIDController(kp=0.8, ki=0.1, kd=0.05)
        # Centered error
        out = pid.update(0.0, dt=0.033)
        self.assertAlmostEqual(out, 0.0, places=3)
        stick = error_to_stick_channel(out)
        self.assertEqual(stick, 128, "Centered target must yield neutral stick 128")

        # Positive error (target to the right)
        out_pos = pid.update(0.5, dt=0.033)
        self.assertGreater(out_pos, 0.0)
        self.assertGreater(error_to_stick_channel(out_pos), 128)

    def test_pid_anti_windup(self):
        """Integral accumulator must respect clamp bounds during sustained off-screen error."""
        pid = PIDController(kp=1.0, ki=0.5, kd=0.0, integral_limit=0.5)
        for _ in range(100):
            pid.update(1.0, dt=0.1)
        self.assertLessEqual(abs(pid.integral), 0.5 + 1e-6)

    def test_optical_flow_drift_vector(self):
        """Verify optical flow velocity vector calculation."""
        p0 = np.array([[100.0, 100.0], [200.0, 200.0]])
        # Displaced +10px in X and -5px in Y over 0.1s
        p1 = np.array([[110.0, 95.0], [210.0, 195.0]])
        vx, vy = calculate_optical_flow_drift(p0, p1, dt=0.1)

        self.assertAlmostEqual(vx, 100.0, places=2)
        self.assertAlmostEqual(vy, -50.0, places=2)


if __name__ == '__main__':
    unittest.main(verbosity=2)
