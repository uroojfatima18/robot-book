#!/usr/bin/env python3
"""
Heuristic Selector

Chapter 5: Adaptive Robotics
Selects behaviors using weighted scoring for smarter decision-making.

Features:
- Configurable weights per behavior
- Deterministic tie-breaking (alphabetical)
- Score explanation for debugging
- Integration with adaptation memory

Usage:
    from heuristic_selector import HeuristicSelector

    weights = {'explore': 1.0, 'avoid': 2.0, 'backup': 1.5}
    selector = HeuristicSelector(weights)

    scores = {'explore': 0.8, 'avoid': 0.3, 'backup': 0.1}
    selected = selector.select(scores)
"""

from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, field


@dataclass
class SelectionResult:
    """
    Result of a behavior selection.

    Attributes:
        selected: Name of selected behavior
        raw_scores: Original scores before weighting
        weighted_scores: Scores after applying weights
        ranking: Behaviors sorted by weighted score
    """
    selected: str
    raw_scores: Dict[str, float]
    weighted_scores: Dict[str, float]
    ranking: List[Tuple[str, float, float]] = field(default_factory=list)


class HeuristicSelector:
    """
    Selects behaviors using weighted scoring.

    Each behavior gets a raw score based on current conditions (0.0 to 1.0).
    Scores are multiplied by configurable weights to determine final selection.
    Ties are broken deterministically using alphabetical ordering.

    Example:
        >>> selector = HeuristicSelector({'explore': 1.0, 'avoid': 2.0})
        >>> selector.select({'explore': 0.5, 'avoid': 0.3})
        'avoid'  # Because 0.3 * 2.0 = 0.6 > 0.5 * 1.0 = 0.5
    """

    def __init__(self, weights: Dict[str, float],
                 tie_break: str = "alphabetical"):
        """
        Initialize with behavior weights.

        Args:
            weights: Dictionary mapping behavior names to weights.
                    Higher weight = more likely to be selected.
                    Missing behaviors default to weight 1.0.
            tie_break: Strategy for breaking ties.
                      "alphabetical" - First alphabetically (default)
                      "random" - Random selection
                      "priority" - Use weight as priority
        """
        self.weights = weights.copy()
        self.tie_break = tie_break
        self.default_weight = 1.0

        # State tracking
        self.last_result: Optional[SelectionResult] = None
        self.selection_count = 0

    def select(self, scores: Dict[str, float]) -> str:
        """
        Select best behavior based on weighted scores.

        Args:
            scores: Raw scores for each behavior (0.0 to 1.0 recommended)

        Returns:
            Name of selected behavior

        Raises:
            ValueError: If scores dict is empty
        """
        if not scores:
            raise ValueError("Scores dictionary cannot be empty")

        self.selection_count += 1

        # Calculate weighted scores
        weighted = {}
        for behavior, score in scores.items():
            weight = self.weights.get(behavior, self.default_weight)
            weighted[behavior] = score * weight

        # Find maximum weighted score
        max_score = max(weighted.values())

        # Get all behaviors with max score (for tie-breaking)
        candidates = [
            b for b, s in weighted.items()
            if abs(s - max_score) < 1e-9  # Float comparison
        ]

        # Apply tie-breaking
        selected = self._break_tie(candidates)

        # Store result for inspection
        ranking = self._calculate_ranking(scores, weighted)
        self.last_result = SelectionResult(
            selected=selected,
            raw_scores=scores.copy(),
            weighted_scores=weighted.copy(),
            ranking=ranking
        )

        return selected

    def _break_tie(self, candidates: List[str]) -> str:
        """
        Break tie between equally-scored behaviors.

        Args:
            candidates: List of behavior names with equal scores

        Returns:
            Selected behavior name
        """
        if len(candidates) == 1:
            return candidates[0]

        if self.tie_break == "alphabetical":
            candidates.sort()
            return candidates[0]
        elif self.tie_break == "priority":
            # Use weight as priority (higher weight wins)
            return max(candidates,
                       key=lambda b: self.weights.get(b, self.default_weight))
        elif self.tie_break == "random":
            import random
            return random.choice(candidates)
        else:
            # Default to alphabetical
            candidates.sort()
            return candidates[0]

    def _calculate_ranking(self, raw: Dict[str, float],
                           weighted: Dict[str, float]) -> List[Tuple[str, float, float]]:
        """
        Calculate behavior ranking by weighted score.

        Returns:
            List of (behavior, raw_score, weighted_score) sorted descending
        """
        ranking = []
        for behavior in raw:
            r = raw[behavior]
            w = weighted.get(behavior, 0)
            ranking.append((behavior, r, w))

        ranking.sort(key=lambda x: (-x[2], x[0]))  # Descending score, then alpha
        return ranking

    def get_ranking(self) -> List[Tuple[str, float, float]]:
        """
        Get behaviors ranked by last selection's weighted score.

        Returns:
            List of (behavior, raw_score, weighted_score) tuples,
            sorted by weighted score descending.
        """
        if self.last_result is None:
            return []
        return self.last_result.ranking

    def explain_selection(self) -> str:
        """
        Generate human-readable explanation of last selection.

        Returns:
            Multi-line explanation string
        """
        if self.last_result is None:
            return "No selection made yet"

        lines = ["Behavior Selection Ranking:", "-" * 40]

        for i, (behavior, raw, weighted) in enumerate(self.last_result.ranking, 1):
            weight = self.weights.get(behavior, self.default_weight)
            marker = "►" if behavior == self.last_result.selected else " "
            lines.append(
                f"{marker} {i}. {behavior:12s}: "
                f"{raw:.3f} × {weight:.1f} = {weighted:.3f}"
            )

        lines.append("-" * 40)
        lines.append(f"Selected: {self.last_result.selected}")

        return "\n".join(lines)

    def update_weight(self, behavior: str, new_weight: float):
        """
        Update weight for a specific behavior.

        Args:
            behavior: Behavior name
            new_weight: New weight value (should be > 0)
        """
        if new_weight <= 0:
            raise ValueError(f"Weight must be positive, got {new_weight}")
        self.weights[behavior] = new_weight

    def adjust_weight(self, behavior: str, delta: float,
                      min_weight: float = 0.1, max_weight: float = 10.0):
        """
        Adjust weight by a delta amount.

        Args:
            behavior: Behavior name
            delta: Amount to add to current weight (can be negative)
            min_weight: Minimum allowed weight
            max_weight: Maximum allowed weight
        """
        current = self.weights.get(behavior, self.default_weight)
        new_weight = max(min_weight, min(max_weight, current + delta))
        self.weights[behavior] = new_weight

    def get_weights(self) -> Dict[str, float]:
        """Get current weights dictionary."""
        return self.weights.copy()

    def reset_weights(self, weights: Dict[str, float]):
        """Reset weights to new values."""
        self.weights = weights.copy()


def score_by_distance(distance: float,
                      safe_distance: float = 1.0,
                      danger_distance: float = 0.2) -> Dict[str, float]:
    """
    Generate behavior scores based on obstacle distance.

    Args:
        distance: Distance to nearest obstacle (meters)
        safe_distance: Distance considered completely safe
        danger_distance: Distance considered dangerous

    Returns:
        Dictionary of behavior scores (0.0 to 1.0)
    """
    # Normalize distance to 0-1 range (1 = safe, 0 = dangerous)
    range_size = safe_distance - danger_distance
    if range_size <= 0:
        normalized = 1.0 if distance >= safe_distance else 0.0
    else:
        normalized = max(0.0, min(1.0,
            (distance - danger_distance) / range_size))

    return {
        'explore': normalized,
        'avoid': 1.0 - normalized,
        'backup': 1.0 if distance < danger_distance else max(0.0, 0.5 - normalized),
        'idle': 0.1
    }


def score_multi_directional(front: float, left: float, right: float,
                           safe_distance: float = 1.0) -> Dict[str, float]:
    """
    Generate scores from multiple directional sensors.

    Args:
        front: Front distance (meters)
        left: Left distance (meters)
        right: Right distance (meters)
        safe_distance: Reference safe distance

    Returns:
        Dictionary of behavior scores
    """
    # Normalize each direction
    def norm(d):
        return min(1.0, d / safe_distance)

    f, l, r = norm(front), norm(left), norm(right)

    return {
        'explore': f * 0.8 + 0.2,
        'avoid': max(0.0, 1.0 - f),
        'turn_left': l * (1.0 - r) if l > r else 0.0,
        'turn_right': r * (1.0 - l) if r > l else 0.0,
        'backup': max(0.0, 1.0 - max(f, l, r)),
        'idle': 0.1
    }


def main(args=None):
    """Demo the heuristic selector."""
    print("Heuristic Selector Demo")
    print("=" * 50)

    # Create selector with weights
    weights = {
        'explore': 1.0,
        'avoid': 2.0,
        'backup': 1.5,
        'idle': 0.3
    }
    selector = HeuristicSelector(weights)

    # Test scenarios
    scenarios = [
        ("Clear path (1.5m)", 1.5),
        ("Getting close (0.6m)", 0.6),
        ("Too close (0.3m)", 0.3),
        ("Danger zone (0.15m)", 0.15),
    ]

    for name, distance in scenarios:
        print(f"\n{name}")
        print("-" * 40)

        scores = score_by_distance(distance)
        selected = selector.select(scores)

        print(selector.explain_selection())

    # Demonstrate weight adjustment
    print("\n" + "=" * 50)
    print("After doubling 'explore' weight:")
    selector.adjust_weight('explore', 1.0)

    scores = score_by_distance(0.6)
    selector.select(scores)
    print(selector.explain_selection())


if __name__ == '__main__':
    main()
