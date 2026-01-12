#!/usr/bin/env python3
"""
Unit Tests for Heuristic Selector

Chapter 5: Adaptive Robotics
Tests weighted scoring and behavior selection logic.
"""

import pytest
import sys
import os

# Add src to path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../src/adaptive_robotics'))

from adaptive_robotics.heuristic_selector import (
    HeuristicSelector, SelectionResult,
    score_by_distance, score_multi_directional
)


class TestHeuristicSelector:
    """Test cases for HeuristicSelector class."""

    @pytest.fixture
    def basic_weights(self):
        """Basic weight configuration."""
        return {
            'explore': 1.0,
            'avoid': 2.0,
            'backup': 1.5,
            'idle': 0.5
        }

    @pytest.fixture
    def selector(self, basic_weights):
        """Create selector with basic weights."""
        return HeuristicSelector(basic_weights)

    def test_initialization(self, selector, basic_weights):
        """Test selector initializes correctly."""
        assert selector.weights == basic_weights
        assert selector.tie_break == "alphabetical"
        assert selector.selection_count == 0

    def test_basic_selection(self, selector):
        """Test basic behavior selection."""
        scores = {'explore': 0.8, 'avoid': 0.3, 'backup': 0.1, 'idle': 0.1}
        selected = selector.select(scores)

        # explore: 0.8 * 1.0 = 0.8
        # avoid: 0.3 * 2.0 = 0.6
        # backup: 0.1 * 1.5 = 0.15
        # idle: 0.1 * 0.5 = 0.05
        assert selected == 'explore'

    def test_weight_influence(self, selector):
        """Test that weights influence selection."""
        # With equal raw scores, higher weight should win
        scores = {'explore': 0.5, 'avoid': 0.5, 'backup': 0.5, 'idle': 0.5}
        selected = selector.select(scores)

        # avoid has highest weight (2.0), so it should win
        # avoid: 0.5 * 2.0 = 1.0
        # backup: 0.5 * 1.5 = 0.75
        # explore: 0.5 * 1.0 = 0.5
        # idle: 0.5 * 0.5 = 0.25
        assert selected == 'avoid'

    def test_high_score_beats_high_weight(self, selector):
        """Test that sufficiently high score can beat higher weight."""
        scores = {'explore': 1.0, 'avoid': 0.4, 'backup': 0.2, 'idle': 0.1}
        selected = selector.select(scores)

        # explore: 1.0 * 1.0 = 1.0
        # avoid: 0.4 * 2.0 = 0.8
        assert selected == 'explore'

    def test_alphabetical_tie_breaking(self):
        """Test alphabetical tie-breaking."""
        selector = HeuristicSelector(
            {'alpha': 1.0, 'beta': 1.0, 'gamma': 1.0},
            tie_break="alphabetical"
        )
        scores = {'alpha': 0.5, 'beta': 0.5, 'gamma': 0.5}
        selected = selector.select(scores)

        assert selected == 'alpha'

    def test_priority_tie_breaking(self):
        """Test priority-based tie-breaking."""
        selector = HeuristicSelector(
            {'alpha': 1.0, 'beta': 2.0, 'gamma': 1.5},
            tie_break="priority"
        )
        # All have same weighted score
        scores = {'alpha': 1.0, 'beta': 0.5, 'gamma': 0.667}
        # alpha: 1.0 * 1.0 = 1.0
        # beta: 0.5 * 2.0 = 1.0
        # gamma: 0.667 * 1.5 ≈ 1.0
        selected = selector.select(scores)

        # beta has highest weight, should win tie
        assert selected == 'beta'

    def test_empty_scores_raises_error(self, selector):
        """Test that empty scores raises ValueError."""
        with pytest.raises(ValueError):
            selector.select({})

    def test_missing_weight_uses_default(self):
        """Test that missing weights default to 1.0."""
        selector = HeuristicSelector({'known': 2.0})
        scores = {'known': 0.4, 'unknown': 0.5}
        selected = selector.select(scores)

        # known: 0.4 * 2.0 = 0.8
        # unknown: 0.5 * 1.0 = 0.5
        assert selected == 'known'

    def test_selection_count_increments(self, selector):
        """Test that selection count increments."""
        scores = {'explore': 0.5, 'avoid': 0.5}

        assert selector.selection_count == 0
        selector.select(scores)
        assert selector.selection_count == 1
        selector.select(scores)
        assert selector.selection_count == 2

    def test_last_result_stored(self, selector):
        """Test that last result is stored."""
        scores = {'explore': 0.8, 'avoid': 0.3}
        selector.select(scores)

        assert selector.last_result is not None
        assert selector.last_result.selected == 'explore'
        assert 'explore' in selector.last_result.raw_scores
        assert 'explore' in selector.last_result.weighted_scores

    def test_get_ranking(self, selector):
        """Test ranking retrieval."""
        scores = {'explore': 0.8, 'avoid': 0.6, 'backup': 0.2, 'idle': 0.1}
        selector.select(scores)

        ranking = selector.get_ranking()

        # Should be sorted by weighted score descending
        assert len(ranking) == 4
        assert ranking[0][0] == 'avoid'  # 0.6 * 2.0 = 1.2
        assert ranking[1][0] == 'explore'  # 0.8 * 1.0 = 0.8

    def test_explain_selection(self, selector):
        """Test explanation generation."""
        scores = {'explore': 0.8, 'avoid': 0.3}
        selector.select(scores)

        explanation = selector.explain_selection()

        assert 'explore' in explanation
        assert 'avoid' in explanation
        assert 'Selected' in explanation

    def test_update_weight(self, selector):
        """Test weight update."""
        selector.update_weight('explore', 3.0)
        assert selector.weights['explore'] == 3.0

    def test_update_weight_invalid(self, selector):
        """Test that invalid weight raises error."""
        with pytest.raises(ValueError):
            selector.update_weight('explore', 0.0)
        with pytest.raises(ValueError):
            selector.update_weight('explore', -1.0)

    def test_adjust_weight(self, selector):
        """Test weight adjustment."""
        original = selector.weights['explore']
        selector.adjust_weight('explore', 0.5)
        assert selector.weights['explore'] == original + 0.5

    def test_adjust_weight_bounds(self, selector):
        """Test weight adjustment respects bounds."""
        selector.adjust_weight('explore', 100.0, max_weight=5.0)
        assert selector.weights['explore'] == 5.0

        selector.adjust_weight('explore', -100.0, min_weight=0.5)
        assert selector.weights['explore'] == 0.5

    def test_get_weights(self, selector, basic_weights):
        """Test get_weights returns copy."""
        weights = selector.get_weights()
        assert weights == basic_weights

        # Modifying returned dict shouldn't affect selector
        weights['explore'] = 999.0
        assert selector.weights['explore'] == basic_weights['explore']

    def test_reset_weights(self, selector):
        """Test reset_weights."""
        new_weights = {'a': 1.0, 'b': 2.0}
        selector.reset_weights(new_weights)
        assert selector.weights == new_weights


class TestScoringFunctions:
    """Test scoring helper functions."""

    def test_score_by_distance_safe(self):
        """Test scoring at safe distance."""
        scores = score_by_distance(1.5)  # Well beyond safe
        assert scores['explore'] > 0.8
        assert scores['avoid'] < 0.2

    def test_score_by_distance_danger(self):
        """Test scoring at dangerous distance."""
        scores = score_by_distance(0.15)  # Below danger
        assert scores['explore'] < 0.2
        assert scores['avoid'] > 0.8
        assert scores['backup'] > 0.5

    def test_score_by_distance_middle(self):
        """Test scoring at middle distance."""
        scores = score_by_distance(0.6)  # Between danger and safe
        # Should have moderate scores
        assert 0.3 < scores['explore'] < 0.7
        assert 0.3 < scores['avoid'] < 0.7

    def test_score_multi_directional_clear_front(self):
        """Test multi-directional scoring with clear front."""
        scores = score_multi_directional(1.5, 0.5, 0.5)
        assert scores['explore'] > scores['avoid']

    def test_score_multi_directional_blocked_front(self):
        """Test multi-directional scoring with blocked front."""
        scores = score_multi_directional(0.3, 1.0, 1.0)
        assert scores['avoid'] > scores['explore']

    def test_score_multi_directional_turn_preference(self):
        """Test that robot prefers clearer side."""
        # Left is clearer
        scores = score_multi_directional(0.3, 1.5, 0.5)
        assert scores['turn_left'] > scores['turn_right']

        # Right is clearer
        scores = score_multi_directional(0.3, 0.5, 1.5)
        assert scores['turn_right'] > scores['turn_left']


class TestSelectionResult:
    """Test SelectionResult dataclass."""

    def test_creation(self):
        """Test SelectionResult creation."""
        result = SelectionResult(
            selected='avoid',
            raw_scores={'avoid': 0.5},
            weighted_scores={'avoid': 1.0},
            ranking=[('avoid', 0.5, 1.0)]
        )
        assert result.selected == 'avoid'
        assert result.raw_scores == {'avoid': 0.5}


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
