"""Tests for gesture_definitions.py - gesture classification logic."""
import sys
import os
import unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from drone_gesture.gesture_definitions import (
    GestureID, GESTURE_NAMES, classify_gesture, is_finger_extended, is_thumb_extended
)


class FakeLandmark:
    def __init__(self, x, y, z=0.0):
        self.x = x
        self.y = y
        self.z = z


def make_landmarks(thumb_tip_x=0.5, thumb_tip_y=0.3,
                   fingers_extended=True):
    """Create fake 21 landmarks for testing."""
    lms = [FakeLandmark(0.5, 0.5)] * 21

    # Wrist
    lms[0] = FakeLandmark(0.5, 0.8)

    # Thumb (lateral motion)
    lms[1] = FakeLandmark(0.45, 0.75)  # CMC
    lms[2] = FakeLandmark(0.42, 0.65)  # MCP
    lms[3] = FakeLandmark(0.40, 0.55)  # IP
    lms[4] = FakeLandmark(thumb_tip_x, thumb_tip_y)  # TIP

    if fingers_extended:
        # Index finger extended (tip above pip, collinear)
        lms[5] = FakeLandmark(0.45, 0.6)   # MCP
        lms[6] = FakeLandmark(0.45, 0.45)  # PIP
        lms[7] = FakeLandmark(0.45, 0.35)  # DIP
        lms[8] = FakeLandmark(0.45, 0.25)  # TIP

        # Middle finger extended
        lms[9] = FakeLandmark(0.5, 0.6)
        lms[10] = FakeLandmark(0.5, 0.45)
        lms[11] = FakeLandmark(0.5, 0.35)
        lms[12] = FakeLandmark(0.5, 0.25)

        # Ring finger extended
        lms[13] = FakeLandmark(0.55, 0.6)
        lms[14] = FakeLandmark(0.55, 0.45)
        lms[15] = FakeLandmark(0.55, 0.35)
        lms[16] = FakeLandmark(0.55, 0.25)

        # Pinky finger extended
        lms[17] = FakeLandmark(0.6, 0.6)
        lms[18] = FakeLandmark(0.6, 0.45)
        lms[19] = FakeLandmark(0.6, 0.35)
        lms[20] = FakeLandmark(0.6, 0.25)
    else:
        # All fingers curled (tip below pip, angle not collinear)
        for base in [5, 9, 13, 17]:
            lms[base] = FakeLandmark(0.5, 0.6)     # MCP
            lms[base+1] = FakeLandmark(0.5, 0.55)   # PIP
            lms[base+2] = FakeLandmark(0.52, 0.58)  # DIP (angled)
            lms[base+3] = FakeLandmark(0.53, 0.62)  # TIP (below PIP)

    return lms


class TestGestureID(unittest.TestCase):
    def test_enum_values(self):
        assert GestureID.NONE == 0
        assert GestureID.OPEN_PALM == 1
        assert GestureID.FIST == 2
        assert GestureID.THUMBS_UP == 3
        assert GestureID.OK_SIGN == 4

    def test_gesture_names(self):
        assert GESTURE_NAMES[GestureID.NONE] == 'none'
        assert GESTURE_NAMES[GestureID.OPEN_PALM] == 'open_palm'
        assert GESTURE_NAMES[GestureID.FIST] == 'fist'


class TestIsFingerExtended(unittest.TestCase):
    def test_extended_collinear(self):
        """Collinear vertical fingers should be extended."""
        lms = make_landmarks(fingers_extended=True)
        assert is_finger_extended(lms, 8, 6, 5) == True

    def test_curled(self):
        """Curled fingers (tip below PIP, angled) should not be extended."""
        lms = make_landmarks(fingers_extended=False)
        assert is_finger_extended(lms, 8, 6, 5) == False

    def test_tip_at_pip(self):
        """Tip at same height as PIP should not be extended."""
        lms = [FakeLandmark(0.5, 0.5)] * 21
        lms[5] = FakeLandmark(0.5, 0.6)   # MCP
        lms[6] = FakeLandmark(0.5, 0.45)  # PIP
        lms[7] = FakeLandmark(0.5, 0.45)  # DIP = PIP
        lms[8] = FakeLandmark(0.5, 0.45)  # TIP = PIP
        # tip.y == pip.y → y_extended is False
        assert is_finger_extended(lms, 8, 6, 5) == False


class TestClassifyGesture(unittest.TestCase):
    def test_none_landmarks(self):
        assert classify_gesture(None) == GestureID.NONE

    def test_too_few_landmarks(self):
        assert classify_gesture([FakeLandmark(0.5, 0.5)] * 10) == GestureID.NONE

    def test_open_palm(self):
        lms = make_landmarks(thumb_tip_x=0.3, thumb_tip_y=0.3,
                             fingers_extended=True)
        assert classify_gesture(lms) == GestureID.OPEN_PALM

    def test_fist(self):
        lms = make_landmarks(thumb_tip_x=0.48, thumb_tip_y=0.75,
                             fingers_extended=False)
        assert classify_gesture(lms) == GestureID.FIST

    def test_thumbs_up(self):
        lms = make_landmarks(thumb_tip_x=0.3, thumb_tip_y=0.3,
                             fingers_extended=False)
        result = classify_gesture(lms)
        assert result == GestureID.THUMBS_UP

    def test_ok_sign_lenient(self):
        """OK sign should be detected with pinch distance < 0.08."""
        lms = make_landmarks(thumb_tip_x=0.48, thumb_tip_y=0.75,
                             fingers_extended=False)
        # Set up pinch: thumb and index tips close together
        # Both tips near PIP level so index registers as curled
        lms[4] = FakeLandmark(0.505, 0.555)  # thumb tip near index PIP
        # Index finger: tip near PIP (not extended)
        lms[5] = FakeLandmark(0.5, 0.6)    # MCP
        lms[6] = FakeLandmark(0.5, 0.55)   # PIP
        lms[7] = FakeLandmark(0.51, 0.56)  # DIP (slightly below)
        lms[8] = FakeLandmark(0.505, 0.555) # TIP (at PIP level, curled)
        # Make 3 other fingers extended
        for base in [9, 13, 17]:
            lms[base] = FakeLandmark(0.5, 0.6)
            lms[base+1] = FakeLandmark(0.5, 0.45)
            lms[base+2] = FakeLandmark(0.5, 0.35)
            lms[base+3] = FakeLandmark(0.5, 0.25)
        # Pinch dist = sqrt(0.005^2 + 0.005^2) ≈ 0.007 < 0.08
        result = classify_gesture(lms)
        assert result == GestureID.OK_SIGN


class TestIsThumbExtended(unittest.TestCase):
    def test_extended(self):
        """Thumb tip far from wrist = extended."""
        lms = [FakeLandmark(0.5, 0.5)] * 21
        lms[0] = FakeLandmark(0.5, 0.8)   # wrist
        lms[3] = FakeLandmark(0.40, 0.55)  # IP
        lms[4] = FakeLandmark(0.30, 0.30)  # TIP (far from wrist)
        assert is_thumb_extended(lms) == True

    def test_curled(self):
        """Thumb tip close to wrist = not extended."""
        lms = [FakeLandmark(0.5, 0.5)] * 21
        lms[0] = FakeLandmark(0.5, 0.8)   # wrist
        lms[3] = FakeLandmark(0.40, 0.55)  # IP
        lms[4] = FakeLandmark(0.48, 0.72)  # TIP (close to wrist)
        assert is_thumb_extended(lms) == False


if __name__ == '__main__':
    unittest.main()
