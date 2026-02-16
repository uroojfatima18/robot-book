"""
Adaptive Robotics Package

Chapter 5: Making robots that respond to change

This package provides ROS 2 nodes for teaching adaptive robotics concepts:
- Behavior switching based on sensor thresholds
- Decision logging for debugging and auditing
- Heuristic behavior selection with weighted scoring
- Adaptive memory for learning from experience

Modules:
    behavior_switcher: Core behavior switching logic with hysteresis
    decision_logger: JSON-based decision audit trails
    heuristic_selector: Weighted scoring for behavior selection
    adaptation_memory: Session-scoped learning from outcomes
    log_viewer: Utility for viewing and filtering decision logs

Example:
    # Launch behavior switcher
    ros2 run adaptive_robotics behavior_switcher

    # Launch with custom parameters
    ros2 run adaptive_robotics behavior_switcher --ros-args \\
        --params-file config/adaptive_params.yaml
"""

__version__ = '0.1.0'
__author__ = 'Chapter Author'

# Package exports
__all__ = [
    'behavior_switcher',
    'decision_logger',
    'heuristic_selector',
    'adaptation_memory',
    'log_viewer',
]
