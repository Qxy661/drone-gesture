"""Tests for gesture_velocity_controller.py - filters and velocity estimation."""
import sys
import os
import unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import time

try:
    from drone_gesture.gesture_velocity_controller import (
        LowPassFilter, DeadZoneFilter, HandVelocityEstimator
    )
    HAS_VELOCITY = True
except ImportError:
    HAS_VELOCITY = False


@unittest.skipUnless(HAS_VELOCITY, "rclpy not available")
class TestLowPassFilter(unittest.TestCase):
    def test_initial_state(self):
        f = LowPassFilter(alpha=0.3, dim=3)
        assert f.state == [0.0, 0.0, 0.0]

    def test_step_response(self):
        f = LowPassFilter(alpha=0.5, dim=3)
        result = f.update([1.0, 1.0, 1.0])
        assert result == [0.5, 0.5, 0.5]

    def test_convergence(self):
        f = LowPassFilter(alpha=0.3, dim=3)
        for _ in range(100):
            result = f.update([1.0, 0.0, -1.0])
        assert abs(result[0] - 1.0) < 0.01
        assert abs(result[1] - 0.0) < 0.01
        assert abs(result[2] - (-1.0)) < 0.01

    def test_reset(self):
        f = LowPassFilter(alpha=0.5, dim=3)
        f.update([1.0, 1.0, 1.0])
        f.reset()
        assert f.state == [0.0, 0.0, 0.0]

    def test_smoothing(self):
        """Lower alpha = more smoothing."""
        f_smooth = LowPassFilter(alpha=0.1, dim=1)
        f_fast = LowPassFilter(alpha=0.9, dim=1)
        f_smooth.update([1.0])
        f_fast.update([1.0])
        assert f_smooth.state[0] < f_fast.state[0]


@unittest.skipUnless(HAS_VELOCITY, "rclpy not available")
class TestDeadZoneFilter(unittest.TestCase):
    def test_within_deadzone(self):
        dz = DeadZoneFilter(threshold=0.01)
        assert dz.apply(0.005) == 0.0
        assert dz.apply(-0.005) == 0.0

    def test_outside_deadzone(self):
        dz = DeadZoneFilter(threshold=0.01)
        result = dz.apply(0.05)
        assert abs(result - 0.04) < 1e-6  # 0.05 - 0.01

    def test_negative_values(self):
        dz = DeadZoneFilter(threshold=0.01)
        result = dz.apply(-0.05)
        assert abs(result - (-0.04)) < 1e-6

    def test_zero(self):
        dz = DeadZoneFilter(threshold=0.01)
        assert dz.apply(0.0) == 0.0

    def test_continuity_at_threshold(self):
        """Output should be continuous at threshold boundary."""
        dz = DeadZoneFilter(threshold=0.01)
        just_inside = dz.apply(0.0099)
        just_outside = dz.apply(0.0101)
        assert just_inside == 0.0
        assert just_outside > 0.0
        assert just_outside < 0.001  # small but non-zero


@unittest.skipUnless(HAS_VELOCITY, "rclpy not available")
class TestHandVelocityEstimator(unittest.TestCase):
    def test_initial_update(self):
        est = HandVelocityEstimator(smoothing=0.4)
        vx, vy, vz = est.update(0.5, 0.5, 0.5, 1.0)
        assert (vx, vy, vz) == (0.0, 0.0, 0.0)

    def test_velocity_estimation(self):
        est = HandVelocityEstimator(smoothing=1.0)  # no smoothing
        est.update(0.0, 0.0, 0.0, 1.0)
        vx, vy, vz = est.update(0.1, 0.0, 0.0, 1.1)
        assert abs(vx - 1.0) < 0.01  # 0.1 / 0.1 = 1.0
        assert abs(vy - 0.0) < 0.01

    def test_smoothing_effect(self):
        est = HandVelocityEstimator(smoothing=0.3)
        est.update(0.0, 0.0, 0.0, 1.0)
        vx, _, _ = est.update(1.0, 0.0, 0.0, 1.1)
        # With smoothing=0.3, velocity should be less than raw (10.0)
        assert vx < 10.0
        assert vx > 0.0

    def test_reset(self):
        est = HandVelocityEstimator()
        est.update(0.0, 0.0, 0.0, 1.0)
        est.update(1.0, 1.0, 1.0, 1.1)
        est.reset()
        assert est.prev_pos is None
        vx, vy, vz = est.update(0.5, 0.5, 0.5, 2.0)
        assert (vx, vy, vz) == (0.0, 0.0, 0.0)

    def test_dt_too_small(self):
        """Should ignore updates with dt < 0.001."""
        est = HandVelocityEstimator(smoothing=1.0)
        est.update(0.0, 0.0, 0.0, 1.0)
        vx, vy, vz = est.update(1.0, 0.0, 0.0, 1.00001)
        # dt too small, should return previous velocity
        assert vx == 0.0


if __name__ == '__main__':
    unittest.main()
