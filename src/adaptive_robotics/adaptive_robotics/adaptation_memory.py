#!/usr/bin/env python3
"""
Adaptation Memory

Chapter 5: Adaptive Robotics
Session-scoped memory for learning from behavior outcomes.

Features:
- Tracks history of behavior executions and outcomes
- Maintains accumulated score adjustments per behavior
- Applies decay to prevent runaway learning
- Enforces bounds for system stability
- Exports state to JSON for analysis

Usage:
    from adaptation_memory import AdaptationMemory

    memory = AdaptationMemory()
    memory.record_outcome('avoid', success=True)
    adjustment = memory.get_adjustment('avoid')
"""

from typing import Dict, List, Optional
from dataclasses import dataclass
from datetime import datetime
import json
import os


@dataclass
class HistoryEntry:
    """
    Record of a single behavior execution outcome.

    Attributes:
        sequence: Sequential number of this outcome
        behavior: Name of the behavior that was executed
        success: Whether the behavior achieved its goal
        adjustment: The adjustment value applied
        timestamp: ISO format timestamp when recorded
    """
    sequence: int
    behavior: str
    success: bool
    adjustment: float
    timestamp: str


class AdaptationMemory:
    """
    Session-scoped memory for learning from behavior outcomes.

    This class tracks the outcomes of behavior executions and maintains
    accumulated adjustments that can be used to modify behavior selection.
    It implements several mechanisms to ensure stable learning:

    - **Bounds**: Adjustments are clamped to prevent extreme values
    - **Decay**: Old outcomes gradually lose influence
    - **History limit**: Only recent outcomes are kept

    The memory is "session-scoped" meaning it resets when the robot restarts.
    This keeps learning manageable and allows fresh starts.

    Example:
        >>> memory = AdaptationMemory(success_adj=0.1, failure_adj=-0.1)
        >>> memory.record_outcome('avoid', success=True)
        0.1
        >>> memory.record_outcome('avoid', success=False)
        -0.1
        >>> memory.get_adjustment('avoid')
        0.0
    """

    def __init__(self,
                 success_adj: float = 0.1,
                 failure_adj: float = -0.1,
                 decay_rate: float = 0.95,
                 decay_interval: int = 10,
                 max_history: int = 100,
                 max_adjustment: float = 1.0,
                 min_adjustment: float = -1.0):
        """
        Initialize adaptation memory.

        Args:
            success_adj: Adjustment value for successful outcomes.
                        Positive values increase future selection probability.
            failure_adj: Adjustment value for failed outcomes.
                        Typically negative to decrease selection probability.
            decay_rate: Multiplier applied during decay (0.0 to 1.0).
                       Values closer to 1.0 mean slower decay.
            decay_interval: Apply decay every N records.
            max_history: Maximum number of history entries to keep.
                        Oldest entries are removed when limit exceeded.
            max_adjustment: Upper bound on accumulated adjustments.
            min_adjustment: Lower bound on accumulated adjustments.
        """
        # Learning parameters
        self.success_adj = success_adj
        self.failure_adj = failure_adj
        self.decay_rate = decay_rate
        self.decay_interval = decay_interval
        self.max_history = max_history
        self.max_adjustment = max_adjustment
        self.min_adjustment = min_adjustment

        # State
        self.history: List[HistoryEntry] = []
        self.score_adjustments: Dict[str, float] = {}
        self.sequence = 0
        self.decay_counter = 0

        # Statistics
        self.total_successes = 0
        self.total_failures = 0

    def record_outcome(self, behavior: str, success: bool) -> float:
        """
        Record a behavior outcome and update adjustments.

        This is the primary method for learning. Each time a behavior
        completes, call this method with whether it succeeded or failed.

        Args:
            behavior: Name of the behavior that was executed
            success: True if behavior achieved its goal, False otherwise

        Returns:
            The adjustment value that was applied
        """
        self.sequence += 1

        # Determine adjustment
        adjustment = self.success_adj if success else self.failure_adj

        # Update statistics
        if success:
            self.total_successes += 1
        else:
            self.total_failures += 1

        # Update accumulated adjustment with bounds
        current = self.score_adjustments.get(behavior, 0.0)
        new_value = current + adjustment
        new_value = max(self.min_adjustment,
                       min(self.max_adjustment, new_value))
        self.score_adjustments[behavior] = new_value

        # Record in history
        entry = HistoryEntry(
            sequence=self.sequence,
            behavior=behavior,
            success=success,
            adjustment=adjustment,
            timestamp=datetime.now().isoformat()
        )
        self.history.append(entry)

        # Trim history if needed
        if len(self.history) > self.max_history:
            self.history = self.history[-self.max_history:]

        # Apply decay periodically
        self.decay_counter += 1
        if self.decay_counter >= self.decay_interval:
            self.apply_decay()
            self.decay_counter = 0

        return adjustment

    def apply_decay(self):
        """
        Apply decay factor to all adjustments.

        This gradually reduces the influence of old outcomes,
        allowing the robot to re-learn if conditions change.
        Called automatically every decay_interval records.
        """
        for behavior in self.score_adjustments:
            self.score_adjustments[behavior] *= self.decay_rate

    def get_adjustment(self, behavior: str) -> float:
        """
        Get the current adjustment for a behavior.

        Args:
            behavior: Behavior name

        Returns:
            Current accumulated adjustment (may be positive or negative)
        """
        return self.score_adjustments.get(behavior, 0.0)

    def get_all_adjustments(self) -> Dict[str, float]:
        """
        Get all current adjustments.

        Returns:
            Dictionary mapping behavior names to adjustments
        """
        return self.score_adjustments.copy()

    def get_success_rate(self, behavior: str,
                         window: Optional[int] = None) -> float:
        """
        Calculate success rate for a specific behavior.

        Args:
            behavior: Behavior name
            window: Optional number of recent entries to consider.
                   If None, uses all history.

        Returns:
            Success rate as float (0.0 to 1.0).
            Returns 0.0 if no history for this behavior.
        """
        entries = [e for e in self.history if e.behavior == behavior]

        if window is not None:
            entries = entries[-window:]

        if not entries:
            return 0.0

        successes = sum(1 for e in entries if e.success)
        return successes / len(entries)

    def get_failure_count(self, behavior: str,
                          consecutive: bool = False) -> int:
        """
        Get failure count for a behavior.

        Args:
            behavior: Behavior name
            consecutive: If True, count only consecutive recent failures

        Returns:
            Number of failures
        """
        entries = [e for e in self.history if e.behavior == behavior]

        if consecutive:
            # Count consecutive failures from most recent
            count = 0
            for entry in reversed(entries):
                if not entry.success:
                    count += 1
                else:
                    break
            return count
        else:
            return sum(1 for e in entries if not e.success)

    def should_adjust_threshold(self, behavior: str,
                                failure_threshold: int = 3) -> bool:
        """
        Check if threshold adjustment is recommended.

        A behavior might need threshold adjustment if it's
        failing repeatedly despite being selected.

        Args:
            behavior: Behavior name
            failure_threshold: Number of consecutive failures to trigger

        Returns:
            True if threshold adjustment is recommended
        """
        return self.get_failure_count(behavior, consecutive=True) >= failure_threshold

    def get_statistics(self) -> Dict:
        """
        Get comprehensive statistics about memory state.

        Returns:
            Dictionary with:
            - total_records: Total outcomes recorded
            - total_successes: Total successful outcomes
            - total_failures: Total failed outcomes
            - overall_success_rate: Global success rate
            - history_size: Current history length
            - behaviors: Per-behavior statistics
        """
        stats = {
            'total_records': self.sequence,
            'total_successes': self.total_successes,
            'total_failures': self.total_failures,
            'overall_success_rate': (
                self.total_successes / self.sequence
                if self.sequence > 0 else 0.0
            ),
            'history_size': len(self.history),
            'behaviors': {}
        }

        # Per-behavior statistics
        behaviors = set(e.behavior for e in self.history)
        for behavior in behaviors:
            entries = [e for e in self.history if e.behavior == behavior]
            successes = sum(1 for e in entries if e.success)

            stats['behaviors'][behavior] = {
                'count': len(entries),
                'successes': successes,
                'failures': len(entries) - successes,
                'success_rate': successes / len(entries) if entries else 0.0,
                'current_adjustment': self.get_adjustment(behavior),
                'consecutive_failures': self.get_failure_count(behavior, consecutive=True)
            }

        return stats

    def print_statistics(self):
        """Print formatted statistics to stdout."""
        stats = self.get_statistics()

        print("\n" + "=" * 50)
        print("ADAPTATION MEMORY STATISTICS")
        print("=" * 50)
        print(f"Total records: {stats['total_records']}")
        print(f"Successes: {stats['total_successes']} | "
              f"Failures: {stats['total_failures']}")
        print(f"Overall success rate: {stats['overall_success_rate']:.1%}")
        print(f"History size: {stats['history_size']}/{self.max_history}")

        print("\nPer-Behavior Statistics:")
        print("-" * 50)

        for behavior, bstats in sorted(stats['behaviors'].items()):
            adj = bstats['current_adjustment']
            adj_bar = "+" * int(adj * 5) if adj > 0 else "-" * int(-adj * 5)
            print(f"\n  {behavior}:")
            print(f"    Count: {bstats['count']} "
                  f"({bstats['successes']}✓ / {bstats['failures']}✗)")
            print(f"    Success rate: {bstats['success_rate']:.1%}")
            print(f"    Adjustment: {adj:+.3f} [{adj_bar}]")
            if bstats['consecutive_failures'] > 0:
                print(f"    ⚠ Consecutive failures: {bstats['consecutive_failures']}")

        print("=" * 50)

    def reset(self):
        """Reset all memory state to initial values."""
        self.history.clear()
        self.score_adjustments.clear()
        self.sequence = 0
        self.decay_counter = 0
        self.total_successes = 0
        self.total_failures = 0

    def export_to_json(self, filepath: str):
        """
        Export memory state to JSON file.

        Args:
            filepath: Path to output file
        """
        data = {
            'sequence': self.sequence,
            'total_successes': self.total_successes,
            'total_failures': self.total_failures,
            'adjustments': self.score_adjustments,
            'history': [
                {
                    'sequence': e.sequence,
                    'behavior': e.behavior,
                    'success': e.success,
                    'adjustment': e.adjustment,
                    'timestamp': e.timestamp
                }
                for e in self.history
            ],
            'config': {
                'success_adj': self.success_adj,
                'failure_adj': self.failure_adj,
                'decay_rate': self.decay_rate,
                'decay_interval': self.decay_interval,
                'max_history': self.max_history,
                'max_adjustment': self.max_adjustment,
                'min_adjustment': self.min_adjustment
            },
            'exported_at': datetime.now().isoformat()
        }

        os.makedirs(os.path.dirname(filepath) or '.', exist_ok=True)
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)

    @classmethod
    def load_from_json(cls, filepath: str) -> 'AdaptationMemory':
        """
        Load memory state from JSON file.

        Args:
            filepath: Path to input file

        Returns:
            AdaptationMemory instance with loaded state
        """
        with open(filepath, 'r') as f:
            data = json.load(f)

        config = data.get('config', {})
        memory = cls(
            success_adj=config.get('success_adj', 0.1),
            failure_adj=config.get('failure_adj', -0.1),
            decay_rate=config.get('decay_rate', 0.95),
            decay_interval=config.get('decay_interval', 10),
            max_history=config.get('max_history', 100),
            max_adjustment=config.get('max_adjustment', 1.0),
            min_adjustment=config.get('min_adjustment', -1.0)
        )

        # Restore state
        memory.sequence = data.get('sequence', 0)
        memory.total_successes = data.get('total_successes', 0)
        memory.total_failures = data.get('total_failures', 0)
        memory.score_adjustments = data.get('adjustments', {})

        # Restore history
        for entry_data in data.get('history', []):
            entry = HistoryEntry(
                sequence=entry_data['sequence'],
                behavior=entry_data['behavior'],
                success=entry_data['success'],
                adjustment=entry_data['adjustment'],
                timestamp=entry_data['timestamp']
            )
            memory.history.append(entry)

        return memory


def main(args=None):
    """Demo the adaptation memory."""
    print("Adaptation Memory Demo")
    print("=" * 50)

    memory = AdaptationMemory(
        success_adj=0.1,
        failure_adj=-0.15,
        decay_rate=0.95
    )

    # Simulate learning for 'avoid' behavior
    print("\nSimulating 'avoid' behavior outcomes...")
    outcomes = [True, True, True, False, True, True, False, True, True, True]
    for success in outcomes:
        adj = memory.record_outcome('avoid', success)
        print(f"  Outcome: {'SUCCESS' if success else 'FAILURE'} | "
              f"Adjustment: {adj:+.2f} | "
              f"Total: {memory.get_adjustment('avoid'):+.3f}")

    # Simulate 'explore' behavior
    print("\nSimulating 'explore' behavior outcomes...")
    for _ in range(5):
        memory.record_outcome('explore', success=True)
    for _ in range(3):
        memory.record_outcome('explore', success=False)

    # Print statistics
    memory.print_statistics()

    # Export to JSON
    export_path = "demo_memory.json"
    memory.export_to_json(export_path)
    print(f"\nExported to: {export_path}")


if __name__ == '__main__':
    main()
