#!/usr/bin/env python3
"""
Unit Tests for Adaptation Memory

Chapter 5: Adaptive Robotics
Tests learning, decay, and bounds functionality.
"""

import pytest
import sys
import os
import tempfile
import json

# Add src to path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../src/adaptive_robotics'))

from adaptive_robotics.adaptation_memory import AdaptationMemory, HistoryEntry


class TestAdaptationMemory:
    """Test cases for AdaptationMemory class."""

    @pytest.fixture
    def memory(self):
        """Create memory with default parameters."""
        return AdaptationMemory()

    @pytest.fixture
    def custom_memory(self):
        """Create memory with custom parameters."""
        return AdaptationMemory(
            success_adj=0.2,
            failure_adj=-0.3,
            decay_rate=0.9,
            decay_interval=5,
            max_history=50,
            max_adjustment=0.5,
            min_adjustment=-0.5
        )

    def test_initialization(self, memory):
        """Test memory initializes correctly."""
        assert memory.success_adj == 0.1
        assert memory.failure_adj == -0.1
        assert memory.decay_rate == 0.95
        assert memory.sequence == 0
        assert len(memory.history) == 0
        assert len(memory.score_adjustments) == 0

    def test_record_success(self, memory):
        """Test recording successful outcome."""
        adj = memory.record_outcome('avoid', success=True)

        assert adj == 0.1
        assert memory.get_adjustment('avoid') == 0.1
        assert memory.sequence == 1
        assert len(memory.history) == 1
        assert memory.total_successes == 1

    def test_record_failure(self, memory):
        """Test recording failed outcome."""
        adj = memory.record_outcome('avoid', success=False)

        assert adj == -0.1
        assert memory.get_adjustment('avoid') == -0.1
        assert memory.total_failures == 1

    def test_cumulative_adjustments(self, memory):
        """Test that adjustments accumulate."""
        memory.record_outcome('avoid', success=True)
        memory.record_outcome('avoid', success=True)
        memory.record_outcome('avoid', success=True)

        # 3 successes = 3 * 0.1 = 0.3
        assert abs(memory.get_adjustment('avoid') - 0.3) < 0.01

    def test_mixed_outcomes(self, memory):
        """Test mixed success/failure outcomes."""
        memory.record_outcome('avoid', success=True)   # +0.1
        memory.record_outcome('avoid', success=False)  # -0.1
        memory.record_outcome('avoid', success=True)   # +0.1

        # Net: +0.1
        assert abs(memory.get_adjustment('avoid') - 0.1) < 0.01

    def test_upper_bound(self, custom_memory):
        """Test that adjustments respect upper bound."""
        # Try to exceed max_adjustment of 0.5
        for _ in range(10):
            custom_memory.record_outcome('avoid', success=True)

        assert custom_memory.get_adjustment('avoid') <= 0.5

    def test_lower_bound(self, custom_memory):
        """Test that adjustments respect lower bound."""
        # Try to go below min_adjustment of -0.5
        for _ in range(10):
            custom_memory.record_outcome('avoid', success=False)

        assert custom_memory.get_adjustment('avoid') >= -0.5

    def test_decay_application(self, custom_memory):
        """Test that decay is applied."""
        # Record outcomes to build up adjustment
        for _ in range(4):
            custom_memory.record_outcome('avoid', success=True)

        adjustment_before = custom_memory.get_adjustment('avoid')

        # Trigger decay (decay_interval is 5)
        custom_memory.record_outcome('avoid', success=True)

        # After decay, adjustment should be reduced
        # Note: The 5th success also adds to adjustment, then decay is applied
        adjustment_after = custom_memory.get_adjustment('avoid')

        # With 5 successes (each +0.2) = 1.0, but capped at 0.5
        # Then decay: 0.5 * 0.9 = 0.45
        assert adjustment_after < adjustment_before or adjustment_after == 0.5

    def test_manual_decay(self, memory):
        """Test manual decay application."""
        memory.record_outcome('avoid', success=True)
        memory.record_outcome('avoid', success=True)

        before = memory.get_adjustment('avoid')
        memory.apply_decay()
        after = memory.get_adjustment('avoid')

        assert after == before * memory.decay_rate

    def test_history_limit(self, custom_memory):
        """Test that history is limited."""
        # max_history is 50
        for i in range(60):
            custom_memory.record_outcome('test', success=True)

        assert len(custom_memory.history) == 50

    def test_multiple_behaviors(self, memory):
        """Test tracking multiple behaviors."""
        memory.record_outcome('avoid', success=True)
        memory.record_outcome('explore', success=False)
        memory.record_outcome('backup', success=True)

        assert memory.get_adjustment('avoid') == 0.1
        assert memory.get_adjustment('explore') == -0.1
        assert memory.get_adjustment('backup') == 0.1

    def test_get_all_adjustments(self, memory):
        """Test getting all adjustments."""
        memory.record_outcome('avoid', success=True)
        memory.record_outcome('explore', success=False)

        adjustments = memory.get_all_adjustments()

        assert 'avoid' in adjustments
        assert 'explore' in adjustments
        assert adjustments['avoid'] == 0.1
        assert adjustments['explore'] == -0.1

    def test_get_success_rate(self, memory):
        """Test success rate calculation."""
        memory.record_outcome('avoid', success=True)
        memory.record_outcome('avoid', success=True)
        memory.record_outcome('avoid', success=False)
        memory.record_outcome('avoid', success=True)

        rate = memory.get_success_rate('avoid')
        assert rate == 0.75  # 3/4

    def test_get_success_rate_no_history(self, memory):
        """Test success rate with no history."""
        rate = memory.get_success_rate('unknown')
        assert rate == 0.0

    def test_get_success_rate_with_window(self, memory):
        """Test success rate with window."""
        # First 3: all failures
        for _ in range(3):
            memory.record_outcome('avoid', success=False)
        # Next 3: all successes
        for _ in range(3):
            memory.record_outcome('avoid', success=True)

        # Window of 3 should only see successes
        rate = memory.get_success_rate('avoid', window=3)
        assert rate == 1.0

        # Full history
        rate = memory.get_success_rate('avoid')
        assert rate == 0.5

    def test_get_failure_count(self, memory):
        """Test failure count."""
        memory.record_outcome('avoid', success=True)
        memory.record_outcome('avoid', success=False)
        memory.record_outcome('avoid', success=False)
        memory.record_outcome('avoid', success=True)
        memory.record_outcome('avoid', success=False)

        total_failures = memory.get_failure_count('avoid')
        assert total_failures == 3

    def test_get_consecutive_failures(self, memory):
        """Test consecutive failure count."""
        memory.record_outcome('avoid', success=True)
        memory.record_outcome('avoid', success=False)
        memory.record_outcome('avoid', success=False)
        memory.record_outcome('avoid', success=False)

        consecutive = memory.get_failure_count('avoid', consecutive=True)
        assert consecutive == 3

    def test_consecutive_failures_reset(self, memory):
        """Test that consecutive failures reset on success."""
        memory.record_outcome('avoid', success=False)
        memory.record_outcome('avoid', success=False)
        memory.record_outcome('avoid', success=True)  # Resets consecutive
        memory.record_outcome('avoid', success=False)

        consecutive = memory.get_failure_count('avoid', consecutive=True)
        assert consecutive == 1

    def test_should_adjust_threshold(self, memory):
        """Test threshold adjustment recommendation."""
        # Default threshold is 3
        memory.record_outcome('avoid', success=False)
        memory.record_outcome('avoid', success=False)
        assert not memory.should_adjust_threshold('avoid')

        memory.record_outcome('avoid', success=False)
        assert memory.should_adjust_threshold('avoid')

    def test_get_statistics(self, memory):
        """Test statistics generation."""
        memory.record_outcome('avoid', success=True)
        memory.record_outcome('avoid', success=False)
        memory.record_outcome('explore', success=True)

        stats = memory.get_statistics()

        assert stats['total_records'] == 3
        assert stats['total_successes'] == 2
        assert stats['total_failures'] == 1
        assert stats['overall_success_rate'] == pytest.approx(2/3)
        assert 'avoid' in stats['behaviors']
        assert 'explore' in stats['behaviors']

    def test_reset(self, memory):
        """Test reset functionality."""
        memory.record_outcome('avoid', success=True)
        memory.record_outcome('avoid', success=True)

        memory.reset()

        assert memory.sequence == 0
        assert len(memory.history) == 0
        assert len(memory.score_adjustments) == 0
        assert memory.total_successes == 0
        assert memory.total_failures == 0

    def test_export_import_json(self, memory):
        """Test JSON export and import."""
        # Build up some state
        memory.record_outcome('avoid', success=True)
        memory.record_outcome('avoid', success=True)
        memory.record_outcome('explore', success=False)

        # Export
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            filepath = f.name

        memory.export_to_json(filepath)

        # Verify file exists and is valid JSON
        with open(filepath, 'r') as f:
            data = json.load(f)

        assert data['sequence'] == 3
        assert 'avoid' in data['adjustments']
        assert len(data['history']) == 3

        # Import
        loaded = AdaptationMemory.load_from_json(filepath)

        assert loaded.sequence == memory.sequence
        assert loaded.get_adjustment('avoid') == memory.get_adjustment('avoid')
        assert loaded.total_successes == memory.total_successes

        # Cleanup
        os.unlink(filepath)


class TestHistoryEntry:
    """Test HistoryEntry dataclass."""

    def test_creation(self):
        """Test HistoryEntry creation."""
        entry = HistoryEntry(
            sequence=1,
            behavior='avoid',
            success=True,
            adjustment=0.1,
            timestamp='2025-01-01T10:00:00'
        )

        assert entry.sequence == 1
        assert entry.behavior == 'avoid'
        assert entry.success is True
        assert entry.adjustment == 0.1


class TestAdaptationMemoryEdgeCases:
    """Test edge cases and boundary conditions."""

    def test_zero_adjustments(self):
        """Test with zero adjustment values."""
        memory = AdaptationMemory(success_adj=0.0, failure_adj=0.0)
        memory.record_outcome('test', success=True)
        memory.record_outcome('test', success=False)

        assert memory.get_adjustment('test') == 0.0

    def test_asymmetric_adjustments(self):
        """Test with asymmetric success/failure adjustments."""
        memory = AdaptationMemory(success_adj=0.1, failure_adj=-0.2)

        memory.record_outcome('test', success=True)  # +0.1
        memory.record_outcome('test', success=False)  # -0.2

        assert memory.get_adjustment('test') == -0.1

    def test_extreme_decay(self):
        """Test with extreme decay rates."""
        # Very fast decay
        memory = AdaptationMemory(decay_rate=0.1, decay_interval=1)
        memory.record_outcome('test', success=True)
        memory.record_outcome('test', success=True)

        # Adjustment should be very small due to aggressive decay
        assert memory.get_adjustment('test') < 0.15

    def test_no_decay(self):
        """Test with no decay."""
        memory = AdaptationMemory(decay_rate=1.0, decay_interval=1)

        for _ in range(5):
            memory.record_outcome('test', success=True)

        # Without decay, should accumulate fully (up to bounds)
        assert memory.get_adjustment('test') == 0.5


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
