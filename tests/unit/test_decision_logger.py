#!/usr/bin/env python3
"""
Unit Tests for Decision Logger

Chapter 5: Adaptive Robotics
Tests logging, filtering, and summary functionality.
"""

import pytest
import sys
import os
import tempfile
import json

# Add src to path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../src/adaptive_robotics'))

from adaptive_robotics.decision_logger import (
    DecisionLogger, InputRecord, DecisionRecord, OutcomeRecord,
    load_log_file
)


class TestInputRecord:
    """Test InputRecord dataclass."""

    def test_creation(self):
        """Test basic creation."""
        record = InputRecord("lidar", "/scan", 0.45, True)
        assert record.sensor == "lidar"
        assert record.topic == "/scan"
        assert record.value == 0.45
        assert record.valid is True

    def test_default_valid(self):
        """Test that valid defaults to True."""
        record = InputRecord("lidar", "/scan", 0.45)
        assert record.valid is True


class TestDecisionRecord:
    """Test DecisionRecord dataclass."""

    def test_creation(self):
        """Test basic creation."""
        record = DecisionRecord("explore", "avoid", "obstacle", 0.5, 0.7)
        assert record.from_behavior == "explore"
        assert record.to_behavior == "avoid"
        assert record.trigger_rule == "obstacle"
        assert record.activate_threshold == 0.5
        assert record.deactivate_threshold == 0.7


class TestOutcomeRecord:
    """Test OutcomeRecord dataclass."""

    def test_creation(self):
        """Test basic creation."""
        record = OutcomeRecord(0.2, 0.0, True, 1000)
        assert record.command_linear_x == 0.2
        assert record.command_angular_z == 0.0
        assert record.success is True
        assert record.duration_ms == 1000

    def test_default_duration(self):
        """Test that duration_ms defaults to 0."""
        record = OutcomeRecord(0.2, 0.0, True)
        assert record.duration_ms == 0


class TestDecisionLogger:
    """Test DecisionLogger class."""

    @pytest.fixture
    def temp_log_dir(self):
        """Create a temporary directory for logs."""
        with tempfile.TemporaryDirectory() as tmpdir:
            yield tmpdir

    @pytest.fixture
    def logger(self, temp_log_dir):
        """Create a logger with temporary directory."""
        return DecisionLogger(temp_log_dir)

    def test_initialization(self, logger):
        """Test logger initializes correctly."""
        assert logger.sequence == 0
        assert logger.session_id is not None
        assert len(logger.session_id) == 8
        assert logger.log_file is not None
        assert os.path.dirname(logger.log_file) == logger.log_dir

    def test_log_directory_creation(self, temp_log_dir):
        """Test that log directory is created if it doesn't exist."""
        new_dir = os.path.join(temp_log_dir, "new_subdir")
        logger = DecisionLogger(new_dir)
        assert os.path.exists(new_dir)

    def test_log_decision(self, logger):
        """Test logging a single decision."""
        input_rec = InputRecord("lidar", "/scan", 0.45, True)
        decision_rec = DecisionRecord("explore", "avoid", "obstacle", 0.5, 0.7)

        seq = logger.log_decision(input_rec, decision_rec)

        assert seq == 1
        assert logger.sequence == 1
        assert len(logger.entries) == 1

    def test_log_multiple_decisions(self, logger):
        """Test logging multiple decisions."""
        for i in range(5):
            input_rec = InputRecord("lidar", "/scan", 0.5 - i * 0.1, True)
            decision_rec = DecisionRecord("explore", "avoid", "obstacle", 0.5, 0.7)
            seq = logger.log_decision(input_rec, decision_rec)
            assert seq == i + 1

        assert logger.sequence == 5
        assert len(logger.entries) == 5

    def test_log_with_outcome(self, logger):
        """Test logging with outcome record."""
        input_rec = InputRecord("lidar", "/scan", 0.45, True)
        decision_rec = DecisionRecord("explore", "avoid", "obstacle", 0.5, 0.7)
        outcome_rec = OutcomeRecord(0.0, 0.5, True, 1000)

        logger.log_decision(input_rec, decision_rec, outcome_rec)

        assert logger.entries[0].outcome is not None
        assert logger.entries[0].outcome.success is True

    def test_log_file_creation(self, logger):
        """Test that log file is created with entries."""
        input_rec = InputRecord("lidar", "/scan", 0.45, True)
        decision_rec = DecisionRecord("explore", "avoid", "obstacle", 0.5, 0.7)
        logger.log_decision(input_rec, decision_rec)

        assert os.path.exists(logger.log_file)

        # Verify file content is valid JSON
        with open(logger.log_file, 'r') as f:
            content = f.read()
            assert content.strip()  # Not empty
            entry = json.loads(content)
            assert entry['sequence'] == 1

    def test_json_lines_format(self, logger):
        """Test that multiple entries are stored as JSON lines."""
        for i in range(3):
            input_rec = InputRecord("lidar", "/scan", 0.5 - i * 0.1, True)
            decision_rec = DecisionRecord("explore", "avoid", "obstacle", 0.5, 0.7)
            logger.log_decision(input_rec, decision_rec)

        with open(logger.log_file, 'r') as f:
            lines = f.readlines()
            assert len(lines) == 3

            for i, line in enumerate(lines, 1):
                entry = json.loads(line)
                assert entry['sequence'] == i

    def test_get_entries_by_behavior(self, logger):
        """Test filtering entries by behavior."""
        # Log different behaviors
        logger.log_decision(
            InputRecord("lidar", "/scan", 0.45, True),
            DecisionRecord("explore", "avoid", "obstacle", 0.5, 0.7)
        )
        logger.log_decision(
            InputRecord("lidar", "/scan", 0.80, True),
            DecisionRecord("avoid", "explore", "obstacle", 0.5, 0.7)
        )
        logger.log_decision(
            InputRecord("lidar", "/scan", 0.15, True),
            DecisionRecord("explore", "backup", "emergency", 0.2, 0.3)
        )

        avoid_entries = logger.get_entries_by_behavior("avoid")
        assert len(avoid_entries) == 1

        explore_entries = logger.get_entries_by_behavior("explore")
        assert len(explore_entries) == 1

        backup_entries = logger.get_entries_by_behavior("backup")
        assert len(backup_entries) == 1

    def test_get_entries_by_trigger(self, logger):
        """Test filtering entries by trigger rule."""
        logger.log_decision(
            InputRecord("lidar", "/scan", 0.45, True),
            DecisionRecord("explore", "avoid", "obstacle", 0.5, 0.7)
        )
        logger.log_decision(
            InputRecord("lidar", "/scan", 0.15, True),
            DecisionRecord("avoid", "backup", "emergency", 0.2, 0.3)
        )

        obstacle_entries = logger.get_entries_by_trigger("obstacle")
        assert len(obstacle_entries) == 1

        emergency_entries = logger.get_entries_by_trigger("emergency")
        assert len(emergency_entries) == 1

    def test_get_invalid_sensor_entries(self, logger):
        """Test getting entries with invalid sensor data."""
        logger.log_decision(
            InputRecord("lidar", "/scan", 0.45, True),
            DecisionRecord("explore", "avoid", "obstacle", 0.5, 0.7)
        )
        logger.log_decision(
            InputRecord("lidar", "/scan", 0.0, False),
            DecisionRecord("avoid", "idle", "invalid", 0.0, 0.0)
        )

        invalid_entries = logger.get_invalid_sensor_entries()
        assert len(invalid_entries) == 1
        assert invalid_entries[0].input.value == 0.0

    def test_get_summary(self, logger):
        """Test summary statistics."""
        # Log various decisions
        logger.log_decision(
            InputRecord("lidar", "/scan", 0.45, True),
            DecisionRecord("explore", "avoid", "obstacle", 0.5, 0.7)
        )
        logger.log_decision(
            InputRecord("lidar", "/scan", 0.55, True),
            DecisionRecord("avoid", "avoid", "obstacle", 0.5, 0.7)
        )
        logger.log_decision(
            InputRecord("lidar", "/scan", 0.80, True),
            DecisionRecord("avoid", "explore", "obstacle", 0.5, 0.7)
        )

        summary = logger.get_summary()

        assert summary['total_decisions'] == 3
        assert summary['total_switches'] == 2  # explore->avoid, avoid->explore
        assert summary['session_id'] == logger.session_id
        assert 'avoid' in summary['behaviors']
        assert 'explore' in summary['behaviors']
        assert 'obstacle' in summary['triggers']

    def test_update_outcome(self, logger):
        """Test updating outcome for existing entry."""
        input_rec = InputRecord("lidar", "/scan", 0.45, True)
        decision_rec = DecisionRecord("explore", "avoid", "obstacle", 0.5, 0.7)
        seq = logger.log_decision(input_rec, decision_rec)

        outcome = OutcomeRecord(0.0, 0.5, True, 500)
        result = logger.update_outcome(seq, outcome)

        assert result is True
        assert logger.entries[0].outcome is not None
        assert logger.entries[0].outcome.duration_ms == 500

    def test_update_outcome_not_found(self, logger):
        """Test updating outcome for non-existent entry."""
        outcome = OutcomeRecord(0.0, 0.5, True, 500)
        result = logger.update_outcome(999, outcome)
        assert result is False


class TestLoadLogFile:
    """Test load_log_file function."""

    @pytest.fixture
    def temp_log_file(self, tmp_path):
        """Create a temporary log file with sample data."""
        log_file = tmp_path / "test_log.json"
        entries = [
            {"sequence": 1, "timestamp": "2025-01-01T10:00:00"},
            {"sequence": 2, "timestamp": "2025-01-01T10:00:01"},
            {"sequence": 3, "timestamp": "2025-01-01T10:00:02"},
        ]
        with open(log_file, 'w') as f:
            for entry in entries:
                f.write(json.dumps(entry) + '\n')
        return str(log_file)

    def test_load_log_file(self, temp_log_file):
        """Test loading entries from file."""
        entries = load_log_file(temp_log_file)
        assert len(entries) == 3
        assert entries[0]['sequence'] == 1
        assert entries[2]['sequence'] == 3

    def test_load_empty_lines(self, tmp_path):
        """Test handling of empty lines in log file."""
        log_file = tmp_path / "test_log.json"
        with open(log_file, 'w') as f:
            f.write('{"sequence": 1}\n')
            f.write('\n')  # Empty line
            f.write('{"sequence": 2}\n')
            f.write('   \n')  # Whitespace only
            f.write('{"sequence": 3}\n')

        entries = load_log_file(str(log_file))
        assert len(entries) == 3


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
