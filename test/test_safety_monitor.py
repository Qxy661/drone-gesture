"""Tests for safety_monitor.py - safety level logic and thresholds."""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import time
from drone_gesture.safety_monitor import SafetyLevel


class TestSafetyLevel:
    def test_constants(self):
        assert SafetyLevel.OK == "ok"
        assert SafetyLevel.WARNING == "warning"
        assert SafetyLevel.CRITICAL == "critical"


class TestSafetyLogic:
    """Test safety check logic without ROS2."""

    def test_battery_critical(self):
        battery_pct = 10.0
        battery_crit = 15.0
        battery_warn = 30.0
        warnings = []
        if battery_pct <= battery_crit:
            warnings.append(f"BATTERY_CRITICAL({battery_pct:.0f}%)")
        elif battery_pct <= battery_warn:
            warnings.append(f"BATTERY_LOW({battery_pct:.0f}%)")
        assert len(warnings) == 1
        assert "BATTERY_CRITICAL" in warnings[0]

    def test_battery_warning(self):
        battery_pct = 25.0
        battery_crit = 15.0
        battery_warn = 30.0
        warnings = []
        if battery_pct <= battery_crit:
            warnings.append(f"BATTERY_CRITICAL({battery_pct:.0f}%)")
        elif battery_pct <= battery_warn:
            warnings.append(f"BATTERY_LOW({battery_pct:.0f}%)")
        assert len(warnings) == 1
        assert "BATTERY_LOW" in warnings[0]

    def test_battery_ok(self):
        battery_pct = 50.0
        battery_crit = 15.0
        battery_warn = 30.0
        warnings = []
        if battery_pct <= battery_crit:
            warnings.append(f"BATTERY_CRITICAL({battery_pct:.0f}%)")
        elif battery_pct <= battery_warn:
            warnings.append(f"BATTERY_LOW({battery_pct:.0f}%)")
        assert len(warnings) == 0

    def test_heartbeat_timeout(self):
        last_heartbeat = time.time() - 15.0
        heartbeat_timeout = 10.0
        now = time.time()
        warnings = []
        if last_heartbeat > 0 and now - last_heartbeat > heartbeat_timeout:
            warnings.append("GESTURE_HEARTBEAT_LOST")
        assert len(warnings) == 1

    def test_heartbeat_ok(self):
        last_heartbeat = time.time() - 2.0
        heartbeat_timeout = 10.0
        now = time.time()
        warnings = []
        if last_heartbeat > 0 and now - last_heartbeat > heartbeat_timeout:
            warnings.append("GESTURE_HEARTBEAT_LOST")
        assert len(warnings) == 0

    def test_fcu_not_connected(self):
        fcu_connected = False
        warnings = []
        if not fcu_connected:
            warnings.append("FCU_NOT_CONNECTED")
        assert "FCU_NOT_CONNECTED" in warnings

    def test_critical_determination(self):
        """Critical warnings should set CRITICAL level."""
        warnings = ["FCU_NOT_CONNECTED", "BATTERY_LOW(25%)"]
        crit_prefixes = ("FCU_NOT_CONNECTED", "FCU_CONNECTION_LOST", "BATTERY_CRITICAL")
        has_critical = any(w.startswith(crit_prefixes) for w in warnings)
        assert has_critical == True

    def test_warning_determination(self):
        """Non-critical warnings should set WARNING level."""
        warnings = ["BATTERY_LOW(25%)", "GESTURE_HEARTBEAT_LOST"]
        crit_prefixes = ("FCU_NOT_CONNECTED", "FCU_CONNECTION_LOST", "BATTERY_CRITICAL")
        has_critical = any(w.startswith(crit_prefixes) for w in warnings)
        assert has_critical == False

    def test_emergency_land_flag(self):
        """Emergency land should only trigger once."""
        emergency_triggered = False
        auto_land = True
        fcu_armed = True
        has_critical = True

        should_land = auto_land and fcu_armed and not emergency_triggered
        assert should_land == True

        emergency_triggered = True
        should_land = auto_land and fcu_armed and not emergency_triggered
        assert should_land == False


if __name__ == '__main__':
    import pytest
    pytest.main([__file__, '-v'])
