# TriggerRule.msg
# Configuration message for behavior trigger rules

# Rule identification
string rule_id              # Unique identifier
bool enabled                # Is rule active?

# Source configuration
string source_id            # FeedbackSource identifier
string topic_name           # ROS 2 topic to monitor

# Threshold configuration (hysteresis)
float32 activate_threshold  # Value to trigger activation
float32 deactivate_threshold # Value to trigger deactivation
string operator             # Comparison: "<", ">", "<=", ">=", "=="

# Target behavior
string target_behavior      # Behavior to activate when triggered
uint8 priority              # Rule precedence (1-100, lower = higher priority)

# State
bool is_active              # Current trigger state
float32 last_value          # Most recent sensor value
