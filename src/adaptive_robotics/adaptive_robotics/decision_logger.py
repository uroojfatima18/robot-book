#!/usr/bin/env python3
"""
Decision Logger

Chapter 5: Adaptive Robotics
Records robot decisions to JSON files for debugging and auditing.

Features:
- Automatic session management with UUIDs
- JSON-formatted log entries (one per line)
- Query and filter capabilities
- Summary statistics

Usage:
    from decision_logger import DecisionLogger, InputRecord, DecisionRecord

    logger = DecisionLogger()
    logger.log_decision(input_record, decision_record)
"""

import json
import os
from datetime import datetime
from typing import Dict, Any, Optional, List
from dataclasses import dataclass, asdict, field
import uuid


@dataclass
class InputRecord:
    """
    Record of sensor input that triggered a decision.

    Attributes:
        sensor: Type of sensor (e.g., "lidar", "odometry")
        topic: ROS 2 topic name
        value: Sensor reading value
        valid: Whether the reading is valid (not corrupted/out of range)
    """
    sensor: str
    topic: str
    value: float
    valid: bool = True


@dataclass
class DecisionRecord:
    """
    Record of the decision made by the behavior switcher.

    Attributes:
        from_behavior: Previous behavior name
        to_behavior: New behavior name
        trigger_rule: ID of the rule that triggered the switch
        activate_threshold: Threshold that was crossed to activate
        deactivate_threshold: Threshold to deactivate
    """
    from_behavior: str
    to_behavior: str
    trigger_rule: str
    activate_threshold: float
    deactivate_threshold: float


@dataclass
class OutcomeRecord:
    """
    Record of the outcome of executing a behavior.

    Attributes:
        command_linear_x: Linear velocity command sent
        command_angular_z: Angular velocity command sent
        success: Whether the behavior achieved its goal
        duration_ms: How long the behavior ran (milliseconds)
    """
    command_linear_x: float
    command_angular_z: float
    success: bool
    duration_ms: int = 0


@dataclass
class DecisionLogEntry:
    """
    Complete decision log entry combining all records.

    Attributes:
        timestamp: ISO 8601 timestamp when decision was made
        session_id: UUID identifying this run session
        sequence: Sequential number of this decision
        input: Sensor input that triggered decision
        decision: The decision that was made
        outcome: Optional outcome record (can be added later)
    """
    timestamp: str
    session_id: str
    sequence: int
    input: InputRecord
    decision: DecisionRecord
    outcome: Optional[OutcomeRecord] = None


class DecisionLogger:
    """
    Logs robot decisions to JSON files for auditing and debugging.

    Creates a new log file for each session with timestamp and UUID.
    Entries are stored as JSON lines (one JSON object per line) for
    efficient streaming and parsing.

    Example:
        >>> logger = DecisionLogger("logs")
        >>> input_rec = InputRecord("lidar", "/scan", 0.45, True)
        >>> decision_rec = DecisionRecord("explore", "avoid",
        ...     "obstacle", 0.5, 0.7)
        >>> logger.log_decision(input_rec, decision_rec)
        1
    """

    def __init__(self, log_dir: str = "logs"):
        """
        Initialize the decision logger.

        Args:
            log_dir: Directory to store log files. Created if doesn't exist.
        """
        self.log_dir = log_dir
        self.session_id = str(uuid.uuid4())[:8]
        self.sequence = 0
        self.entries: List[DecisionLogEntry] = []
        self.log_file: Optional[str] = None

        # Create log directory
        os.makedirs(log_dir, exist_ok=True)

        # Create log file with timestamp and session ID
        timestamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
        self.log_file = os.path.join(
            log_dir,
            f"decision_log_{timestamp}_{self.session_id}.json"
        )

    def log_decision(self,
                     input_record: InputRecord,
                     decision_record: DecisionRecord,
                     outcome_record: Optional[OutcomeRecord] = None) -> int:
        """
        Log a decision entry.

        Args:
            input_record: Sensor input that triggered the decision
            decision_record: The decision that was made
            outcome_record: Optional outcome (can be added later)

        Returns:
            Sequence number of the logged entry
        """
        self.sequence += 1

        entry = DecisionLogEntry(
            timestamp=datetime.now().isoformat(),
            session_id=self.session_id,
            sequence=self.sequence,
            input=input_record,
            decision=decision_record,
            outcome=outcome_record
        )

        self.entries.append(entry)
        self._write_entry(entry)

        return self.sequence

    def _write_entry(self, entry: DecisionLogEntry):
        """
        Write a single entry to the log file.

        Uses JSON lines format (one JSON object per line).

        Args:
            entry: The entry to write
        """
        entry_dict = {
            'timestamp': entry.timestamp,
            'session_id': entry.session_id,
            'sequence': entry.sequence,
            'input': asdict(entry.input),
            'decision': asdict(entry.decision),
            'outcome': asdict(entry.outcome) if entry.outcome else None
        }

        with open(self.log_file, 'a') as f:
            f.write(json.dumps(entry_dict) + '\n')

    def update_outcome(self, sequence: int, outcome: OutcomeRecord) -> bool:
        """
        Update the outcome for a previously logged decision.

        Note: This updates the in-memory entry but not the file.
        For production use, consider using a database or rewriting the file.

        Args:
            sequence: Sequence number of entry to update
            outcome: Outcome record to add

        Returns:
            True if entry was found and updated, False otherwise
        """
        for entry in self.entries:
            if entry.sequence == sequence:
                entry.outcome = outcome
                return True
        return False

    def get_entries_by_behavior(self, behavior: str) -> List[DecisionLogEntry]:
        """
        Filter entries by target behavior.

        Args:
            behavior: Behavior name to filter by

        Returns:
            List of entries where to_behavior matches
        """
        return [
            e for e in self.entries
            if e.decision.to_behavior == behavior
        ]

    def get_entries_by_trigger(self, trigger_rule: str) -> List[DecisionLogEntry]:
        """
        Filter entries by trigger rule.

        Args:
            trigger_rule: Rule ID to filter by

        Returns:
            List of entries where trigger_rule matches
        """
        return [
            e for e in self.entries
            if e.decision.trigger_rule == trigger_rule
        ]

    def get_invalid_sensor_entries(self) -> List[DecisionLogEntry]:
        """
        Get all entries with invalid sensor data.

        Returns:
            List of entries where input.valid is False
        """
        return [e for e in self.entries if not e.input.valid]

    def get_behavior_switches(self) -> List[DecisionLogEntry]:
        """
        Get only entries where behavior actually changed.

        Returns:
            List of entries where from_behavior != to_behavior
        """
        return [
            e for e in self.entries
            if e.decision.from_behavior != e.decision.to_behavior
        ]

    def get_summary(self) -> Dict[str, Any]:
        """
        Get summary statistics of logged decisions.

        Returns:
            Dictionary with:
            - total_decisions: Total number of logged decisions
            - total_switches: Number of behavior changes
            - session_id: This session's UUID
            - behaviors: Count of entries per behavior
            - triggers: Count of entries per trigger rule
            - invalid_sensor_count: Number of invalid sensor readings
        """
        behavior_counts: Dict[str, int] = {}
        trigger_counts: Dict[str, int] = {}
        switches = 0
        invalid_count = 0

        for entry in self.entries:
            # Count behaviors
            behavior = entry.decision.to_behavior
            behavior_counts[behavior] = behavior_counts.get(behavior, 0) + 1

            # Count triggers
            trigger = entry.decision.trigger_rule
            trigger_counts[trigger] = trigger_counts.get(trigger, 0) + 1

            # Count switches
            if entry.decision.from_behavior != entry.decision.to_behavior:
                switches += 1

            # Count invalid sensors
            if not entry.input.valid:
                invalid_count += 1

        return {
            'total_decisions': len(self.entries),
            'total_switches': switches,
            'session_id': self.session_id,
            'log_file': self.log_file,
            'behaviors': behavior_counts,
            'triggers': trigger_counts,
            'invalid_sensor_count': invalid_count
        }

    def print_summary(self):
        """Print a formatted summary to stdout."""
        summary = self.get_summary()

        print("\n" + "=" * 50)
        print("DECISION LOG SUMMARY")
        print("=" * 50)
        print(f"Session ID: {summary['session_id']}")
        print(f"Log file: {summary['log_file']}")
        print(f"Total decisions: {summary['total_decisions']}")
        print(f"Behavior switches: {summary['total_switches']}")
        print(f"Invalid sensor readings: {summary['invalid_sensor_count']}")

        print("\nBehavior distribution:")
        for behavior, count in sorted(summary['behaviors'].items()):
            pct = 100 * count / max(summary['total_decisions'], 1)
            print(f"  {behavior}: {count} ({pct:.1f}%)")

        print("\nTrigger distribution:")
        for trigger, count in sorted(summary['triggers'].items()):
            print(f"  {trigger}: {count}")

        print("=" * 50 + "\n")


def load_log_file(filepath: str) -> List[Dict]:
    """
    Load entries from a log file.

    Args:
        filepath: Path to the log file

    Returns:
        List of dictionaries representing log entries
    """
    entries = []
    with open(filepath, 'r') as f:
        for line in f:
            if line.strip():
                entries.append(json.loads(line))
    return entries


def main(args=None):
    """Demo the decision logger."""
    print("Decision Logger Demo")
    print("-" * 40)

    # Create logger
    logger = DecisionLogger("demo_logs")
    print(f"Logging to: {logger.log_file}")

    # Log some sample decisions
    input1 = InputRecord("lidar", "/scan", 0.45, True)
    decision1 = DecisionRecord("explore", "avoid", "obstacle", 0.5, 0.7)
    logger.log_decision(input1, decision1)
    print("Logged decision 1: explore -> avoid")

    input2 = InputRecord("lidar", "/scan", 0.72, True)
    decision2 = DecisionRecord("avoid", "explore", "obstacle", 0.5, 0.7)
    logger.log_decision(input2, decision2)
    print("Logged decision 2: avoid -> explore")

    input3 = InputRecord("lidar", "/scan", 0.0, False)
    decision3 = DecisionRecord("explore", "idle", "invalid_sensor", 0.0, 0.0)
    logger.log_decision(input3, decision3)
    print("Logged decision 3: explore -> idle (invalid sensor)")

    # Print summary
    logger.print_summary()


if __name__ == '__main__':
    main()
