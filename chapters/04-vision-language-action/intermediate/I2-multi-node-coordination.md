---
id: i2_multi_node_coordination
title: Multi-Node Workflow Coordination
tier: intermediate
chapter: chapter_4_workflow
estimated_time: 60-90 minutes
prerequisites: ["i1_ros2_state_machines"]
---

# I2: Multi-Node Workflow Coordination

## Learning Objectives

By the end of this lesson, you will be able to:
- Coordinate multiple ROS 2 nodes in a workflow
- Pass data between nodes using topics and services
- Implement producer-consumer patterns
- Handle node dependencies and startup order
- Build a complete multi-node pipeline

## Introduction

Real robotic workflows involve multiple specialized nodes working together. In this lesson, you'll learn to coordinate sensor processing, decision making, and actuation nodes into a cohesive workflow.

We'll build a complete waypoint navigation system with separate nodes for waypoint management, navigation control, and status monitoring - a pattern used in warehouse automation and delivery robots.

## Multi-Node Architecture

A typical workflow consists of specialized nodes:

```
Waypoint Manager → Navigation Controller → Status Monitor
       ↓                    ↓                    ↓
   (Publishes)          (Subscribes)         (Monitors)
   /waypoints           /waypoints           /robot_state
                        /current_pose
```

### Code Example: Waypoint Manager Node

```python
# waypoint_manager.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import yaml

class WaypointManager(Node):
    """Manages and publishes waypoints for navigation"""

    def __init__(self):
        super().__init__('waypoint_manager')

        # Declare parameters
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('loop_waypoints', False)

        # Load waypoints
        waypoint_file = self.get_parameter('waypoint_file').value
        self.waypoints = self.load_waypoints(waypoint_file)
        self.current_index = 0
        self.loop = self.get_parameter('loop_waypoints').value

        # Publishers
        self.waypoint_pub = self.create_publisher(
            PoseStamped,
            '/current_waypoint',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/waypoint_status',
            10
        )

        # Subscribers
        self.goal_reached_sub = self.create_subscription(
            String,
            '/navigation_state',
            self.navigation_state_callback,
            10
        )

        # Timer to publish current waypoint
        self.timer = self.create_timer(1.0, self.publish_current_waypoint)

        self.get_logger().info(
            f'Waypoint Manager initialized with {len(self.waypoints)} waypoints'
        )

    def load_waypoints(self, filename):
        """Load waypoints from YAML file"""
        if not filename:
            # Default waypoints if no file specified
            return [
                {'x': 2.0, 'y': 1.0, 'theta': 0.0},
                {'x': 2.0, 'y': -1.0, 'theta': 1.57},
                {'x': -2.0, 'y': -1.0, 'theta': 3.14},
                {'x': -2.0, 'y': 1.0, 'theta': -1.57},
            ]

        try:
            with open(filename, 'r') as f:
                data = yaml.safe_load(f)
                return data.get('waypoints', [])
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return []

    def publish_current_waypoint(self):
        """Publish the current waypoint"""
        if not self.waypoints:
            return

        waypoint = self.waypoints[self.current_index]

        # Create PoseStamped message
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = waypoint['x']
        msg.pose.position.y = waypoint['y']
        msg.pose.position.z = 0.0

        # Convert theta to quaternion (simplified for 2D)
        import math
        theta = waypoint.get('theta', 0.0)
        msg.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.orientation.w = math.cos(theta / 2.0)

        self.waypoint_pub.publish(msg)

        # Publish status
        status_msg = String()
        status_msg.data = f'Waypoint {self.current_index + 1}/{len(self.waypoints)}'
        self.status_pub.publish(status_msg)

    def navigation_state_callback(self, msg):
        """Handle navigation state updates"""
        if msg.data == 'goal_reached':
            self.get_logger().info(
                f'Waypoint {self.current_index + 1} reached'
            )
            self.advance_to_next_waypoint()

    def advance_to_next_waypoint(self):
        """Move to the next waypoint"""
        self.current_index += 1

        if self.current_index >= len(self.waypoints):
            if self.loop:
                self.get_logger().info('Looping back to first waypoint')
                self.current_index = 0
            else:
                self.get_logger().info('All waypoints completed!')
                self.current_index = len(self.waypoints) - 1

def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Example: Navigation Controller Node

```python
# navigation_controller.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

class NavigationController(Node):
    """Controls navigation to waypoints"""

    def __init__(self):
        super().__init__('navigation_controller')

        # Nav2 action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # Current goal tracking
        self.current_goal = None
        self.goal_handle = None

        # Subscribers
        self.waypoint_sub = self.create_subscription(
            PoseStamped,
            '/current_waypoint',
            self.waypoint_callback,
            10
        )

        # Publishers
        self.state_pub = self.create_publisher(
            String,
            '/navigation_state',
            10
        )

        self.get_logger().info('Navigation Controller initialized')

    def waypoint_callback(self, msg):
        """Handle new waypoint"""
        # Only accept new waypoint if not currently navigating
        if self.goal_handle is None or not self.goal_handle.is_active:
            self.get_logger().info(
                f'New waypoint: ({msg.pose.position.x:.2f}, '
                f'{msg.pose.position.y:.2f})'
            )
            self.send_navigation_goal(msg)

    def send_navigation_goal(self, goal_pose):
        """Send navigation goal to Nav2"""
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 not available')
            self.publish_state('error')
            return

        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # Send goal
        self.get_logger().info('Sending goal to Nav2')
        self.publish_state('navigating')

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance"""
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self.publish_state('goal_rejected')
            return

        self.get_logger().info('Goal accepted')

        # Get result
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result()

        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('Goal reached!')
            self.publish_state('goal_reached')
        else:
            self.get_logger().warn(f'Navigation failed: {result.status}')
            self.publish_state('navigation_failed')

        self.goal_handle = None

    def publish_state(self, state):
        """Publish navigation state"""
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Example: Status Monitor Node

```python
# status_monitor.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from datetime import datetime

class StatusMonitor(Node):
    """Monitors workflow status and logs progress"""

    def __init__(self):
        super().__init__('status_monitor')

        # State tracking
        self.navigation_state = 'unknown'
        self.waypoint_status = 'unknown'
        self.current_waypoint = None

        # Subscribers
        self.nav_state_sub = self.create_subscription(
            String,
            '/navigation_state',
            self.nav_state_callback,
            10
        )

        self.waypoint_status_sub = self.create_subscription(
            String,
            '/waypoint_status',
            self.waypoint_status_callback,
            10
        )

        self.waypoint_sub = self.create_subscription(
            PoseStamped,
            '/current_waypoint',
            self.waypoint_callback,
            10
        )

        # Timer for status updates
        self.timer = self.create_timer(5.0, self.print_status)

        self.get_logger().info('Status Monitor initialized')

    def nav_state_callback(self, msg):
        """Handle navigation state updates"""
        if msg.data != self.navigation_state:
            timestamp = datetime.now().strftime('%H:%M:%S')
            self.get_logger().info(
                f'[{timestamp}] Navigation: {self.navigation_state} -> {msg.data}'
            )
            self.navigation_state = msg.data

    def waypoint_status_callback(self, msg):
        """Handle waypoint status updates"""
        self.waypoint_status = msg.data

    def waypoint_callback(self, msg):
        """Handle current waypoint updates"""
        self.current_waypoint = msg

    def print_status(self):
        """Print current workflow status"""
        self.get_logger().info('=== Workflow Status ===')
        self.get_logger().info(f'Navigation State: {self.navigation_state}')
        self.get_logger().info(f'Waypoint Status: {self.waypoint_status}')

        if self.current_waypoint:
            pos = self.current_waypoint.pose.position
            self.get_logger().info(
                f'Current Goal: ({pos.x:.2f}, {pos.y:.2f})'
            )

def main(args=None):
    rclpy.init(args=args)
    node = StatusMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Service-Based Coordination

For synchronous operations, use services:

### Code Example: Service-Based Control

```python
# waypoint_service.py
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped

class WaypointService(Node):
    """Waypoint manager with service interface"""

    def __init__(self):
        super().__init__('waypoint_service')

        self.waypoints = [
            {'x': 2.0, 'y': 1.0},
            {'x': 2.0, 'y': -1.0},
        ]
        self.current_index = 0

        # Services
        self.next_srv = self.create_service(
            Trigger,
            '/next_waypoint',
            self.next_waypoint_callback
        )

        self.reset_srv = self.create_service(
            Trigger,
            '/reset_waypoints',
            self.reset_waypoints_callback
        )

        # Publisher
        self.waypoint_pub = self.create_publisher(
            PoseStamped,
            '/current_waypoint',
            10
        )

        self.get_logger().info('Waypoint Service ready')

    def next_waypoint_callback(self, request, response):
        """Service to advance to next waypoint"""
        self.current_index += 1

        if self.current_index >= len(self.waypoints):
            response.success = False
            response.message = 'No more waypoints'
            return response

        # Publish new waypoint
        waypoint = self.waypoints[self.current_index]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = waypoint['x']
        msg.pose.position.y = waypoint['y']
        self.waypoint_pub.publish(msg)

        response.success = True
        response.message = f'Waypoint {self.current_index + 1}/{len(self.waypoints)}'
        return response

    def reset_waypoints_callback(self, request, response):
        """Service to reset to first waypoint"""
        self.current_index = 0
        response.success = True
        response.message = 'Waypoints reset'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = WaypointService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**To call the service:**
```bash
# Advance to next waypoint
ros2 service call /next_waypoint std_srvs/srv/Trigger

# Reset waypoints
ros2 service call /reset_waypoints std_srvs/srv/Trigger
```

## Running the Complete Workflow

**Terminal 1: Launch Nav2**
```bash
ros2 launch nav2_bringup tb3_simulation_launch.py
```

**Terminal 2: Waypoint Manager**
```bash
ros2 run workflow_orchestration waypoint_manager
```

**Terminal 3: Navigation Controller**
```bash
ros2 run workflow_orchestration navigation_controller
```

**Terminal 4: Status Monitor**
```bash
ros2 run workflow_orchestration status_monitor
```

## Best Practices

### 1. Node Responsibilities
- Each node should have a single, clear purpose
- Avoid monolithic nodes that do everything
- Use composition for related functionality

### 2. Communication Patterns
- **Topics**: Continuous data streams (sensor data, state)
- **Services**: Request-response (commands, queries)
- **Actions**: Long-running tasks with feedback (navigation)

### 3. Error Handling
- Each node should handle its own errors
- Publish error states for coordination
- Implement timeouts for external dependencies

### 4. Testing
- Test each node independently first
- Test pairs of nodes before full integration
- Use mock nodes for testing

## Hardware Notes

> **Simulation vs. Real Hardware**: Multi-node coordination works the same in simulation and reality, but network latency matters on real robots. Use appropriate QoS settings (RELIABLE for commands, BEST_EFFORT for high-frequency data) and implement timeout handling for production systems.

## Summary

- Multi-node workflows coordinate specialized components
- Use topics for continuous data, services for commands
- Each node should have a clear, focused responsibility
- Test nodes independently before integration
- Implement proper error handling and monitoring

## Exercises

1. **Add Emergency Stop** (Intermediate)
   - Create an emergency stop node
   - Stop navigation when triggered
   - Resume after emergency cleared
   - Acceptance Criteria: Robot stops within 1 second

2. **Implement Waypoint Editing** (Advanced)
   - Add service to insert/remove waypoints
   - Update waypoint list dynamically
   - Persist changes to file
   - Acceptance Criteria: Can modify waypoints without restart

## Next Steps

Continue to [I3: Launch Files and Pipeline Startup](./I3-launch-pipeline-startup.md) to learn how to orchestrate node startup with launch files.
