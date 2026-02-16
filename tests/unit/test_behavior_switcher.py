#!/usr/bin/env python3
"""
Unit Tests for Behavior Switcher

Chapter 5: Adaptive Robotics
Tests hysteresis threshold and behavior switching logic.
"""

import pytest
import sys
import os

# Add src to path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../src/adaptive_robotics'))

from adaptive_robotics.behavior_switcher import HysteresisThreshold


class TestHysteresisThreshold:
    """Test cases for HysteresisThreshold class."""

    def test_initialization(self):
        """Test that HysteresisThreshold initializes correctly."""
        trigger = HysteresisThreshold(activate=0.5, deactivate=0.7)
        assert trigger.activate == 0.5
        assert trigger.deactivate == 0.7
        assert trigger.is_active is False

    def test_invalid_thresholds(self):
        """Test that invalid thresholds raise ValueError."""
        with pytest.raises(ValueError):
            HysteresisThreshold(activate=0.7, deactivate=0.5)

        with pytest.raises(ValueError):
            HysteresisThreshold(activate=0.5, deactivate=0.5)

    def test_activation(self):
        """Test that trigger activates when value drops below threshold."""
        trigger = HysteresisThreshold(activate=0.5, deactivate=0.7)

        # Above threshold - should stay inactive
        assert trigger.update(0.8) is False
        assert trigger.update(0.6) is False

        # Below activate threshold - should become active
        assert trigger.update(0.4) is True
        assert trigger.is_active is True

    def test_deactivation(self):
        """Test that trigger deactivates when value rises above threshold."""
        trigger = HysteresisThreshold(activate=0.5, deactivate=0.7)

        # Activate first
        trigger.update(0.4)
        assert trigger.is_active is True

        # In dead band - should stay active
        assert trigger.update(0.6) is True

        # Above deactivate threshold - should become inactive
        assert trigger.update(0.8) is False
        assert trigger.is_active is False

    def test_dead_band(self):
        """Test that trigger doesn't change state in dead band."""
        trigger = HysteresisThreshold(activate=0.5, deactivate=0.7)

        # Start inactive, value in dead band
        assert trigger.update(0.6) is False  # Stays inactive

        # Activate
        trigger.update(0.4)
        assert trigger.is_active is True

        # In dead band - stays active
        assert trigger.update(0.55) is True
        assert trigger.update(0.65) is True

        # Deactivate
        trigger.update(0.8)
        assert trigger.is_active is False

        # In dead band - stays inactive
        assert trigger.update(0.65) is False
        assert trigger.update(0.55) is False

    def test_boundary_values(self):
        """Test behavior at exact threshold values."""
        trigger = HysteresisThreshold(activate=0.5, deactivate=0.7)

        # At activate threshold (not below) - should stay inactive
        # Note: We use < for activation
        assert trigger.update(0.5) is False

        # Just below activate - should activate
        assert trigger.update(0.49) is True

        # At deactivate threshold (not above) - should stay active
        # Note: We use > for deactivation
        assert trigger.update(0.7) is True

        # Just above deactivate - should deactivate
        assert trigger.update(0.71) is False

    def test_reset(self):
        """Test that reset returns trigger to initial state."""
        trigger = HysteresisThreshold(activate=0.5, deactivate=0.7)

        # Activate
        trigger.update(0.4)
        assert trigger.is_active is True

        # Reset
        trigger.reset()
        assert trigger.is_active is False

    def test_rapid_value_changes(self):
        """Test behavior with rapid value changes (simulating noisy sensor)."""
        trigger = HysteresisThreshold(activate=0.5, deactivate=0.7)

        # Simulate noisy sensor hovering around 0.6 (in dead band)
        values = [0.58, 0.62, 0.59, 0.61, 0.60, 0.58, 0.63]

        # Initially inactive, should stay inactive
        for v in values:
            assert trigger.update(v) is False

        # Now activate
        trigger.update(0.4)

        # Same noisy values, should stay active (in dead band)
        for v in values:
            assert trigger.update(v) is True

    def test_repr(self):
        """Test string representation."""
        trigger = HysteresisThreshold(activate=0.5, deactivate=0.7)
        repr_str = repr(trigger)
        assert "0.5" in repr_str
        assert "0.7" in repr_str
        assert "False" in repr_str


class TestHysteresisIntegration:
    """Integration tests for hysteresis in realistic scenarios."""

    def test_obstacle_approach_scenario(self):
        """Test a realistic obstacle approach scenario."""
        trigger = HysteresisThreshold(activate=0.5, deactivate=0.7)

        # Robot approaching obstacle from 1.0m
        distances = [1.0, 0.9, 0.8, 0.7, 0.6, 0.55, 0.52, 0.48, 0.45]
        expected_active = [False, False, False, False, False, False, False, True, True]

        for dist, expected in zip(distances, expected_active):
            result = trigger.update(dist)
            assert result == expected, f"At distance {dist}: expected {expected}, got {result}"

    def test_obstacle_retreat_scenario(self):
        """Test robot backing away from obstacle."""
        trigger = HysteresisThreshold(activate=0.5, deactivate=0.7)

        # Start active (close to obstacle)
        trigger.update(0.3)
        assert trigger.is_active is True

        # Robot backing away
        distances = [0.35, 0.45, 0.55, 0.65, 0.68, 0.72, 0.8]
        expected_active = [True, True, True, True, True, False, False]

        for dist, expected in zip(distances, expected_active):
            result = trigger.update(dist)
            assert result == expected, f"At distance {dist}: expected {expected}, got {result}"

    def test_oscillation_prevention(self):
        """Verify that hysteresis prevents oscillation."""
        trigger = HysteresisThreshold(activate=0.5, deactivate=0.7)

        # Without hysteresis, this would cause rapid switching
        # With hysteresis, state should be stable
        oscillating_values = [0.48, 0.52, 0.48, 0.52, 0.48, 0.52]

        # First value activates
        trigger.update(0.48)
        assert trigger.is_active is True

        # Count state changes
        state_changes = 0
        prev_state = trigger.is_active

        for v in oscillating_values[1:]:
            trigger.update(v)
            if trigger.is_active != prev_state:
                state_changes += 1
            prev_state = trigger.is_active

        # Should have 0 state changes (all values in dead band)
        assert state_changes == 0, f"Had {state_changes} state changes, expected 0"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
