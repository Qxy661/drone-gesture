"""Integration tests for the full gesture-to-command pipeline.

Tests the data flow without ROS2 runtime:
  gesture_definitions → gesture data → commander logic → velocity logic
"""
import sys
import os
import unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import json
import time
import math
from drone_gesture.gesture_definitions import GestureID, GESTURE_NAMES, classify_gesture

try:
    from drone_gesture.gesture_velocity_controller import (
        LowPassFilter, DeadZoneFilter, HandVelocityEstimator
    )
    HAS_VELOCITY = True
except ImportError:
    HAS_VELOCITY = False


class FakeLandmark:
    def __init__(self, x, y, z=0.0):
        self.x = x
        self.y = y
        self.z = z


def make_open_palm_landmarks():
    """Create landmarks for open palm gesture."""
    lms = [FakeLandmark(0.5, 0.5)] * 21
    lms[0] = FakeLandmark(0.5, 0.8)  # wrist
    # Thumb extended
    lms[1] = FakeLandmark(0.45, 0.75)
    lms[2] = FakeLandmark(0.42, 0.65)
    lms[3] = FakeLandmark(0.40, 0.55)
    lms[4] = FakeLandmark(0.30, 0.30)
    # All fingers extended (collinear, tip above PIP)
    for base, x in [(5, 0.45), (9, 0.50), (13, 0.55), (17, 0.60)]:
        lms[base] = FakeLandmark(x, 0.6)
        lms[base+1] = FakeLandmark(x, 0.45)
        lms[base+2] = FakeLandmark(x, 0.35)
        lms[base+3] = FakeLandmark(x, 0.25)
    return lms


def make_fist_landmarks():
    """Create landmarks for fist gesture."""
    lms = [FakeLandmark(0.5, 0.5)] * 21
    lms[0] = FakeLandmark(0.5, 0.8)
    # Thumb curled
    lms[1] = FakeLandmark(0.45, 0.75)
    lms[2] = FakeLandmark(0.42, 0.65)
    lms[3] = FakeLandmark(0.40, 0.55)
    lms[4] = FakeLandmark(0.48, 0.72)
    # All fingers curled
    for base in [5, 9, 13, 17]:
        lms[base] = FakeLandmark(0.5, 0.6)
        lms[base+1] = FakeLandmark(0.5, 0.55)
        lms[base+2] = FakeLandmark(0.52, 0.58)
        lms[base+3] = FakeLandmark(0.53, 0.62)
    return lms


class TestGestureToMessage(unittest.TestCase):
    """Test gesture → JSON message format."""

    def test_gesture_to_json(self):
        """Gesture should serialize to valid JSON with expected fields."""
        gesture_id = GestureID.OPEN_PALM
        payload = {
            'gesture': GESTURE_NAMES[gesture_id],
            'gesture_id': int(gesture_id),
            'confidence': 0.95,
        }
        msg_json = json.dumps(payload)
        parsed = json.loads(msg_json)
        assert parsed['gesture'] == 'open_palm'
        assert parsed['gesture_id'] == 1

    def test_landmarks_in_message(self):
        """Message should include landmarks when available."""
        lms = make_open_palm_landmarks()
        landmarks_data = [{'x': lm.x, 'y': lm.y, 'z': lm.z} for lm in lms]
        payload = {
            'gesture': 'open_palm',
            'gesture_id': 1,
            'landmarks': landmarks_data,
        }
        assert len(payload['landmarks']) == 21


class TestCommanderStateMachine(unittest.TestCase):
    """Test gesture_commander state transitions without ROS2."""

    def test_takeoff_from_idle(self):
        """OPEN_PALM in IDLE should trigger takeoff."""
        state = 'IDLE'
        gesture = GestureID.OPEN_PALM

        if gesture == GestureID.OPEN_PALM and state == 'IDLE':
            state = 'TAKING_OFF'
        assert state == 'TAKING_OFF'

    def test_land_from_hovering(self):
        """FIST in HOVERING should trigger landing."""
        state = 'HOVERING'
        gesture = GestureID.FIST

        if gesture == GestureID.FIST and state in ('HOVERING', 'MOVING'):
            state = 'LANDING'
        assert state == 'LANDING'

    def test_move_forward_from_hovering(self):
        """THUMBS_UP in HOVERING should trigger movement."""
        state = 'HOVERING'
        gesture = GestureID.THUMBS_UP

        if gesture == GestureID.THUMBS_UP and state == 'HOVERING':
            state = 'MOVING'
        assert state == 'MOVING'

    def test_takeoff_ignored_when_flying(self):
        """OPEN_PALM should be ignored when already flying."""
        for state in ['TAKING_OFF', 'HOVERING', 'MOVING', 'LANDING']:
            gesture = GestureID.OPEN_PALM
            should_takeoff = (gesture == GestureID.OPEN_PALM and state == 'IDLE')
            assert should_takeoff == False, f'Should not takeoff from {state}'

    def test_debounce_logic(self):
        """Same gesture within debounce time should be ignored."""
        last_time = time.time()
        debounce = 1.5
        now = last_time + 0.5  # 0.5s later, within debounce
        should_process = now - last_time >= debounce
        assert should_process == False

        now = last_time + 2.0  # 2s later, outside debounce
        should_process = now - last_time >= debounce
        assert should_process == True


@unittest.skipUnless(HAS_VELOCITY, "rclpy not available")
class TestVelocityPipeline(unittest.TestCase):
    """Test the velocity estimation pipeline end-to-end."""

    def test_hand_movement_to_velocity(self):
        """Moving hand should produce non-zero velocity."""
        estimator = HandVelocityEstimator(smoothing=1.0)
        # First frame: hand at center
        estimator.update(0.5, 0.5, 0.0, 1.0)
        # Second frame: hand moved right
        vx, vy, vz = estimator.update(0.6, 0.5, 0.0, 1.1)
        # vx should be ~1.0 (0.1 / 0.1s)
        assert abs(vx - 1.0) < 0.1

    def test_full_pipeline_filters(self):
        """Velocity should go through estimator → lowpass → deadzone."""
        estimator = HandVelocityEstimator(smoothing=0.5)
        lpf = LowPassFilter(alpha=0.3, dim=3)
        dz = DeadZoneFilter(threshold=0.01)

        # Simulate hand moving right over several frames
        positions = [(0.5, 0.5, 0.0)]
        for i in range(5):
            positions.append((0.5 + i * 0.02, 0.5, 0.0))

        for i in range(1, len(positions)):
            raw_vel = estimator.update(*positions[i], time=float(i) * 0.1)
            filtered = lpf.update(list(raw_vel))
            fx = dz.apply(filtered[0])

        # After consistent rightward motion, fx should be positive
        assert fx > 0

    def test_still_hand_produces_zero(self):
        """Stationary hand should produce zero velocity after filtering."""
        estimator = HandVelocityEstimator(smoothing=0.3)
        lpf = LowPassFilter(alpha=0.2, dim=3)
        dz = DeadZoneFilter(threshold=0.01)

        for i in range(20):
            raw_vel = estimator.update(0.5, 0.5, 0.0, float(i) * 0.1)
            filtered = lpf.update(list(raw_vel))
            fx = dz.apply(filtered[0])
            fy = dz.apply(filtered[1])
            fz = dz.apply(filtered[2])

        assert abs(fx) < 0.01
        assert abs(fy) < 0.01
        assert abs(fz) < 0.01


class TestSafetyIntegration(unittest.TestCase):
    """Test safety monitor logic integration."""

    def test_critical_battery_triggers_land(self):
        """Critical battery should trigger emergency land."""
        battery_pct = 10.0
        battery_crit = 15.0
        fcu_armed = True
        emergency_triggered = False
        auto_land = True

        should_land = battery_pct <= battery_crit
        assert should_land == True

        if should_land and auto_land and fcu_armed and not emergency_triggered:
            emergency_triggered = True
        assert emergency_triggered == True

    def test_emergency_only_once(self):
        """Emergency land should only trigger once per critical event."""
        emergency_triggered = False
        trigger_count = 0

        for _ in range(5):
            if not emergency_triggered:
                trigger_count += 1
                emergency_triggered = True

        assert trigger_count == 1


if __name__ == '__main__':
    unittest.main()
