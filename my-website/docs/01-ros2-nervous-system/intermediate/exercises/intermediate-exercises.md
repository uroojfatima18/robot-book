---
id: intermediate_exercises
title: "Intermediate Tier Exercises"
sidebar_position: 4
tier: intermediate
chapter: chapter_1_ros2
---

# Intermediate Tier Exercises

This document consolidates all exercises from the Intermediate tier lessons for easy reference and self-assessment.

---

## From I1: Nodes, Topics, Services, and Actions

### Exercise I1.1: Sensor Publisher (Easy)

**Objective**: Modify the minimal publisher to simulate sensor data.

**Setup**:
1. Start with the `minimal_publisher.py` from the lesson
2. You'll need: `from std_msgs.msg import Float64`

**Tasks**:
1. Change the message type from `String` to `Float64`
2. Publish simulated temperature readings (random values between 20.0 and 30.0)
3. Use topic name `/sensor/temperature`
4. Add a parameter for the sensor name that appears in log messages

**Code Skeleton**:
```python
import random
from std_msgs.msg import Float64

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        # TODO: Create publisher for Float64 on /sensor/temperature
        # TODO: Create timer
        pass

    def timer_callback(self):
        # TODO: Generate random temperature between 20.0 and 30.0
        # TODO: Publish the value
        pass
```

**Acceptance Criteria**:
- [ ] Node publishes `Float64` messages to `/sensor/temperature`
- [ ] Values are random floats in the 20-30 range
- [ ] `ros2 topic echo /sensor/temperature` shows the values
- [ ] Publishing rate is configurable

<details>
<summary>Solution</summary>

```python
#!/usr/bin/env python3
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')

        self.publisher = self.create_publisher(Float64, '/sensor/temperature', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Temperature publisher started')

    def timer_callback(self):
        msg = Float64()
        msg.data = random.uniform(20.0, 30.0)
        self.publisher.publish(msg)
        self.get_logger().info(f'Temperature: {msg.data:.2f}°C')

def main():
    rclpy.init()
    node = TemperaturePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

</details>

---

### Exercise I1.2: Message Counter and Statistics (Medium)

**Objective**: Create a subscriber that tracks message statistics.

**Tasks**:
1. Subscribe to the `/chatter` topic
2. Track: total messages, messages per second, first/last message content
3. Every 10 messages, log a statistics summary
4. Handle publisher restart gracefully (detect gaps in sequence)

**Statistics to Track**:
```python
class MessageStats:
    total_count: int = 0
    first_message: str = ""
    last_message: str = ""
    start_time: float = 0.0
    messages_per_second: float = 0.0
```

**Acceptance Criteria**:
- [ ] Accurately counts all messages
- [ ] Calculates correct messages/second rate
- [ ] Logs statistics every 10 messages
- [ ] Handles publisher stop/restart

<details>
<summary>Solution</summary>

```python
#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MessageStatsSubscriber(Node):
    def __init__(self):
        super().__init__('message_stats')
        self.subscription = self.create_subscription(
            String, 'chatter', self.callback, 10
        )

        self.total_count = 0
        self.first_message = None
        self.last_message = None
        self.start_time = None

        self.get_logger().info('Statistics subscriber started')

    def callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
            self.first_message = msg.data

        self.total_count += 1
        self.last_message = msg.data

        if self.total_count % 10 == 0:
            elapsed = time.time() - self.start_time
            rate = self.total_count / elapsed if elapsed > 0 else 0

            self.get_logger().info(
                f'Stats: count={self.total_count}, '
                f'rate={rate:.2f} msg/s, '
                f'first="{self.first_message}", '
                f'last="{self.last_message}"'
            )

def main():
    rclpy.init()
    node = MessageStatsSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

</details>

---

### Exercise I1.3: Calculator Service (Medium)

**Objective**: Create a service that performs multiple math operations.

**Tasks**:
1. Create a custom service (or use existing interfaces creatively)
2. Support operations: add, subtract, multiply, divide
3. Handle division by zero gracefully
4. Create both server and client

**Approach**: Since creating custom interfaces requires a package, use `example_interfaces/srv/AddTwoInts` and encode the operation in the request values, or design a workaround.

**Alternative Approach**: Create multiple services (`/add`, `/subtract`, `/multiply`, `/divide`)

**Acceptance Criteria**:
- [ ] All four operations work correctly
- [ ] Division by zero returns error (not crash)
- [ ] Can be called from CLI: `ros2 service call /add ...`
- [ ] Client demonstrates all operations

<details>
<summary>Solution (Multiple Services Approach)</summary>

```python
#!/usr/bin/env python3
"""Calculator with multiple service endpoints."""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class CalculatorServer(Node):
    def __init__(self):
        super().__init__('calculator_server')

        self.add_srv = self.create_service(
            AddTwoInts, 'add', self.add_callback
        )
        self.sub_srv = self.create_service(
            AddTwoInts, 'subtract', self.sub_callback
        )
        self.mul_srv = self.create_service(
            AddTwoInts, 'multiply', self.mul_callback
        )
        self.div_srv = self.create_service(
            AddTwoInts, 'divide', self.div_callback
        )

        self.get_logger().info('Calculator services ready')

    def add_callback(self, req, resp):
        resp.sum = req.a + req.b
        self.get_logger().info(f'{req.a} + {req.b} = {resp.sum}')
        return resp

    def sub_callback(self, req, resp):
        resp.sum = req.a - req.b
        self.get_logger().info(f'{req.a} - {req.b} = {resp.sum}')
        return resp

    def mul_callback(self, req, resp):
        resp.sum = req.a * req.b
        self.get_logger().info(f'{req.a} * {req.b} = {resp.sum}')
        return resp

    def div_callback(self, req, resp):
        if req.b == 0:
            self.get_logger().error('Division by zero!')
            resp.sum = 0  # Indicate error
        else:
            resp.sum = req.a // req.b  # Integer division
            self.get_logger().info(f'{req.a} / {req.b} = {resp.sum}')
        return resp

def main():
    rclpy.init()
    node = CalculatorServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Test with:
```bash
ros2 service call /add example_interfaces/srv/AddTwoInts "{a: 10, b: 5}"
ros2 service call /divide example_interfaces/srv/AddTwoInts "{a: 10, b: 0}"
```

</details>

---

## From I2: Python ROS Bridge (rclpy)

### Exercise I2.1: Parameterized Publisher (Easy)

**Objective**: Add runtime-configurable parameters to a publisher.

**Tasks**:
1. Add `message_prefix` string parameter (default: "Hello")
2. Add `publish_rate` float parameter (default: 1.0)
3. Add `enabled` bool parameter (default: true)
4. Make timer respect `publish_rate` changes
5. Skip publishing when `enabled` is false

**Acceptance Criteria**:
- [ ] Parameters settable at launch: `--ros-args -p publish_rate:=5.0`
- [ ] Parameters changeable at runtime: `ros2 param set ...`
- [ ] Invalid values rejected with error message
- [ ] Timer rate updates when parameter changes

<details>
<summary>Solution</summary>

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult

class ParameterizedPublisher(Node):
    def __init__(self):
        super().__init__('parameterized_publisher')

        # Declare parameters
        self.declare_parameter('message_prefix', 'Hello')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('enabled', True)

        # Get initial values
        self.prefix = self.get_parameter('message_prefix').value
        self.rate = self.get_parameter('publish_rate').value
        self.enabled = self.get_parameter('enabled').value

        # Register callback
        self.add_on_set_parameters_callback(self.param_callback)

        # Create publisher and timer
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        self.count = 0

        self.get_logger().info(f'Started with prefix="{self.prefix}", rate={self.rate}')

    def param_callback(self, params):
        for p in params:
            if p.name == 'publish_rate':
                if p.value <= 0:
                    return SetParametersResult(
                        successful=False,
                        reason='Rate must be positive'
                    )
                self.rate = p.value
                self.destroy_timer(self.timer)
                self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
            elif p.name == 'message_prefix':
                self.prefix = p.value
            elif p.name == 'enabled':
                self.enabled = p.value

        return SetParametersResult(successful=True)

    def timer_callback(self):
        if not self.enabled:
            return

        msg = String()
        msg.data = f'{self.prefix} #{self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.count += 1

def main():
    rclpy.init()
    node = ParameterizedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

</details>

---

### Exercise I2.2: Launch File Creation (Medium)

**Objective**: Create a launch file for publisher and subscriber.

**Tasks**:
1. Launch both `minimal_publisher` and `minimal_subscriber`
2. Add launch argument for topic name
3. Use remapping so both use the configured topic
4. Add launch argument for namespace

**Test Commands**:
```bash
ros2 launch my_package my_launch.py
ros2 launch my_package my_launch.py topic:=my_topic namespace:=robot1
```

**Acceptance Criteria**:
- [ ] Both nodes start with single command
- [ ] Topic name is configurable
- [ ] Namespace works correctly
- [ ] Nodes communicate successfully

<details>
<summary>Solution</summary>

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    topic_arg = DeclareLaunchArgument(
        'topic', default_value='chatter',
        description='Topic for communication'
    )

    ns_arg = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Namespace for nodes'
    )

    publisher = Node(
        package='demo_nodes_cpp',
        executable='talker',
        name='publisher',
        namespace=LaunchConfiguration('namespace'),
        remappings=[('chatter', LaunchConfiguration('topic'))],
        output='screen'
    )

    subscriber = Node(
        package='demo_nodes_cpp',
        executable='listener',
        name='subscriber',
        namespace=LaunchConfiguration('namespace'),
        remappings=[('chatter', LaunchConfiguration('topic'))],
        output='screen'
    )

    return LaunchDescription([
        topic_arg,
        ns_arg,
        publisher,
        subscriber
    ])
```

</details>

---

### Exercise I2.3: Multi-Threaded Service (Challenge)

**Objective**: Create a service with concurrent callback handling.

**Tasks**:
1. Create a service that simulates 3-second processing
2. Use `MultiThreadedExecutor`
3. Add a fast timer (10 Hz) in a reentrant group
4. Verify timer continues during service execution

**Verification**:
- Call service in one terminal
- Watch timer logs continue in node terminal
- Service should complete after 3 seconds

**Acceptance Criteria**:
- [ ] Service blocks for 3 seconds
- [ ] Timer callbacks continue during service execution
- [ ] Multiple concurrent service calls work
- [ ] Clean shutdown on Ctrl+C

<details>
<summary>Solution</summary>

```python
#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.srv import AddTwoInts

class ConcurrentServiceNode(Node):
    def __init__(self):
        super().__init__('concurrent_service')

        # Callback groups
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = ReentrantCallbackGroup()

        # Slow service
        self.srv = self.create_service(
            AddTwoInts, 'slow_add', self.slow_callback,
            callback_group=self.service_group
        )

        # Fast timer
        self.timer = self.create_timer(
            0.1, self.timer_callback,
            callback_group=self.timer_group
        )

        self.tick = 0
        self.get_logger().info('Concurrent service ready')

    def slow_callback(self, req, resp):
        self.get_logger().info(f'Service called: {req.a} + {req.b}')
        self.get_logger().info('Processing (3 seconds)...')
        time.sleep(3.0)
        resp.sum = req.a + req.b
        self.get_logger().info(f'Service complete: {resp.sum}')
        return resp

    def timer_callback(self):
        self.tick += 1
        if self.tick % 10 == 0:
            self.get_logger().info(f'Timer tick: {self.tick}')

def main():
    rclpy.init()
    node = ConcurrentServiceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Test:
```bash
# Terminal 1: Run node
python3 concurrent_service.py

# Terminal 2: Call service (blocks for 3 sec)
ros2 service call /slow_add example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"

# Observe: Timer ticks continue in Terminal 1 during service execution
```

</details>

---

## Self-Assessment Checklist

After completing all intermediate exercises, verify you can:

### rclpy Fundamentals
- [ ] Create nodes that publish and subscribe
- [ ] Use timers for periodic callbacks
- [ ] Handle keyboard interrupts gracefully
- [ ] Log at appropriate levels (debug, info, warn, error)

### Communication Patterns
- [ ] Implement publishers with custom message types
- [ ] Create subscribers with statistics tracking
- [ ] Build service servers and clients
- [ ] Understand when to use topics vs services

### Configuration and Launch
- [ ] Declare and use parameters
- [ ] Handle parameter changes at runtime
- [ ] Validate parameter values
- [ ] Write launch files with arguments and remapping

### Concurrency
- [ ] Choose appropriate executor (single vs multi-threaded)
- [ ] Use callback groups correctly
- [ ] Understand when callbacks can run in parallel

### Ready for Advanced?
If you checked all boxes above, you're ready for the Advanced tier where you'll create URDF robot descriptions and implement action servers!

---

## Next Steps

Continue to [Advanced Tier: URDF & Humanoid Robot Description](../../advanced/01-urdf-humanoid.md)
