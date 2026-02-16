# DecisionLog.msg
# Complete decision record for auditing and replay

# Header with timestamp
std_msgs/Header header

# Session tracking
string session_id           # UUID for current session
uint32 sequence             # Order within session

# Input snapshot
string input_sensor         # Sensor type (e.g., "lidar", "odometry")
string input_topic          # ROS 2 topic name
float32 input_value         # Sensor reading value
bool input_valid            # Data quality flag

# Decision details
string from_behavior        # Previous behavior
string to_behavior          # New/current behavior
string trigger_rule         # Rule that caused decision
float32 activate_threshold  # Threshold for activation
float32 deactivate_threshold # Threshold for deactivation

# Outcome (may be filled later)
float32 cmd_linear_x        # Commanded linear velocity
float32 cmd_angular_z       # Commanded angular velocity
bool outcome_success        # Did behavior succeed?
uint32 duration_ms          # Time in behavior (milliseconds)

# Scoring (for advanced tier)
float32[] behavior_scores   # Raw scores before weighting
float32[] weighted_scores   # Scores after weighting
string[] behavior_names     # Names corresponding to scores
