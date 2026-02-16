# BehaviorSwitch.msg
# Message published when robot switches between behaviors

# Header with timestamp
std_msgs/Header header

# Behavior transition
string from_behavior        # Previous behavior (e.g., "explore")
string to_behavior          # New behavior (e.g., "avoid")

# Trigger information
string trigger_rule_id      # ID of the rule that triggered switch
float32 trigger_value       # Sensor value that triggered switch
float32 threshold_used      # Threshold that was crossed

# Metadata
uint32 sequence             # Decision sequence number
bool is_hysteresis_active   # True if in hysteresis dead band
