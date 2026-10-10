#!/usr/bin/env python3
"""
Test Suite: RC UFO Protocol Calculations & Checksum Verification
Validates packet generation, axis clamping, sensitivity scaling, and bitwise checksums.
"""

import unittest


def compute_gl_checksum(roll: int, pitch: int, throttle: int, yaw: int, flags1: int, flags2: int) -> int:
    """GL Protocol (Type 2, 21-Byte Packet) XOR Checksum formula."""
    return (flags1 ^ (((pitch ^ roll) ^ throttle) ^ yaw)) ^ (flags2 & 0xFF) & 0xFF


def compute_legacy_checksum(roll: int, pitch: int, throttle: int, yaw: int, flags: int) -> int:
    """Legacy Protocol (Type 10, 9-Byte Packet) XOR Checksum formula."""
    return (((roll ^ pitch) ^ throttle) ^ yaw) ^ (flags & 0xFF) & 0xFF


def build_gl_packet(roll: int, pitch: int, throttle: int, yaw: int, flags1: int = 0, flags2: int = 0) -> bytes:
    r = max(1, min(255, int(round(roll))))
    p = max(1, min(255, int(round(pitch))))
    t = max(0, min(255, int(round(throttle))))
    y = max(1, min(255, int(round(yaw))))
    cs = compute_gl_checksum(r, p, t, y, flags1, flags2) & 0xFF

    inner = bytearray(20)
    inner[0] = 0x66  # Header
    inner[1] = 0x14  # Length = 20
    inner[2] = r
    inner[3] = p
    inner[4] = t
    inner[5] = y
    inner[6] = flags1 & 0xFF
    inner[7] = flags2 & 0xFF
    # inner[8..17] are 0x00 reserved
    inner[18] = cs
    inner[19] = 0x99  # Tail

    return bytes([0x03]) + bytes(inner)


def build_legacy_packet(roll: int, pitch: int, throttle: int, yaw: int, flags: int = 0) -> bytes:
    r = max(1, min(255, int(round(roll))))
    p = max(1, min(255, int(round(pitch))))
    t = max(0, min(255, int(round(throttle))))
    y = max(1, min(255, int(round(yaw))))
    cs = compute_legacy_checksum(r, p, t, y, flags) & 0xFF

    return bytes([0x03, 0x66, r, p, t, y, flags & 0xFF, cs, 0x99])


def scale_stick_axis(normalized_val: float, half_len: int, trim: int = 0, trim_step: int = 1) -> int:
    """Calculates 8-bit integer channel from normalized joystick input [-1.0, 1.0]."""
    # Deadzone filter
    if abs(normalized_val) < 0.05:
        normalized_val = 0.0
    val = 128 + (normalized_val * half_len) + (trim * trim_step)
    return max(1, min(255, int(round(val))))


class TestProtocolCalculations(unittest.TestCase):

    def test_gl_neutral_packet(self):
        """Neutral hover (128, 128, 128, 128) with no flags must yield Checksum = 0x00."""
        pkt = build_gl_packet(128, 128, 128, 128, 0, 0)
        self.assertEqual(len(pkt), 21, "GL Packet must be exactly 21 bytes")
        self.assertEqual(pkt[0], 0x03, "Command ID must be 0x03")
        self.assertEqual(pkt[1], 0x66, "Magic header must be 0x66")
        self.assertEqual(pkt[2], 0x14, "Payload length must be 20 (0x14)")
        self.assertEqual(list(pkt[3:7]), [128, 128, 128, 128])
        self.assertEqual(pkt[7], 0x00, "Flags 1 must be 0")
        self.assertEqual(pkt[8], 0x00, "Flags 2 must be 0")
        self.assertEqual(list(pkt[9:19]), [0] * 10, "Reserved bytes must be 0")
        self.assertEqual(pkt[19], 0x00, "128^128^128^128 = 0 Checksum")
        self.assertEqual(pkt[20], 0x99, "End byte must be 0x99")

    def test_legacy_neutral_packet(self):
        """Neutral legacy packet must be 9 bytes with Checksum = 0x00."""
        pkt = build_legacy_packet(128, 128, 128, 128, 0)
        self.assertEqual(len(pkt), 9, "Legacy Packet must be exactly 9 bytes")
        self.assertEqual(pkt[0], 0x03)
        self.assertEqual(pkt[1], 0x66)
        self.assertEqual(list(pkt[2:6]), [128, 128, 128, 128])
        self.assertEqual(pkt[6], 0x00, "Flags must be 0")
        self.assertEqual(pkt[7], 0x00, "Checksum must be 0")
        self.assertEqual(pkt[8], 0x99, "Tail must be 0x99")

    def test_gear_rate_scalings(self):
        """Test stick scaling across 30%, 60%, and 100% sensitivity gears."""
        # 30% Gear (halfLen = 40)
        self.assertEqual(scale_stick_axis(1.0, 40), 168)
        self.assertEqual(scale_stick_axis(-1.0, 40), 88)
        self.assertEqual(scale_stick_axis(0.0, 40), 128)

        # 60% Gear (halfLen = 60)
        self.assertEqual(scale_stick_axis(1.0, 60), 188)
        self.assertEqual(scale_stick_axis(-1.0, 60), 68)

        # 100% Gear (halfLen = 127)
        self.assertEqual(scale_stick_axis(1.0, 127), 255)
        self.assertEqual(scale_stick_axis(-1.0, 127), 1)

    def test_deadzone_filter(self):
        """Stick noise under 0.05 threshold should return centered 128."""
        self.assertEqual(scale_stick_axis(0.04, 127), 128)
        self.assertEqual(scale_stick_axis(-0.03, 127), 128)

    def test_boundary_clamping(self):
        """Values beyond limits must safely clamp without overflow."""
        self.assertEqual(scale_stick_axis(2.5, 127), 255)
        self.assertEqual(scale_stick_axis(-3.0, 127), 1)
        pkt = build_gl_packet(-50, 300, -10, 500)
        self.assertEqual(pkt[3], 1, "Roll lower clamp is 1")
        self.assertEqual(pkt[4], 255, "Pitch upper clamp is 255")
        self.assertEqual(pkt[5], 0, "Throttle lower clamp is 0")
        self.assertEqual(pkt[6], 255, "Yaw upper clamp is 255")

    def test_flags_and_checksum_integrity(self):
        """Test known action flag XOR checksums."""
        # Takeoff flag active (flags1 = 0x01) on neutral sticks
        cs = compute_gl_checksum(128, 128, 128, 128, flags1=0x01, flags2=0x00)
        self.assertEqual(cs, 0x01)

        # Gyro correction (flags1 = 0x04) + Fixed Height (flags2 = 0x02)
        cs = compute_gl_checksum(128, 128, 128, 128, flags1=0x04, flags2=0x02)
        self.assertEqual(cs, 0x04 ^ 0x02)  # 0x06

        # Asymmetric axes: Roll=200, Pitch=100, Throttle=150, Yaw=80, flags1=0x08, flags2=0x01
        expected_cs = (0x08 ^ (((100 ^ 200) ^ 150) ^ 80)) ^ 0x01
        actual_cs = compute_gl_checksum(200, 100, 150, 80, 0x08, 0x01)
        self.assertEqual(actual_cs, expected_cs & 0xFF)


if __name__ == '__main__':
    unittest.main(verbosity=2)
