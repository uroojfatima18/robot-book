#!/usr/bin/env python3
"""
Sensor Streamer - Stream simulation data to external AI systems.

This node collects sensor data from the digital twin and streams it
to external APIs for AI/ML training pipelines.

Topics Subscribed:
    /sim/joint_states - Joint positions and velocities
    /sim/imu/data - IMU readings (if available)
    /sim/camera/image_raw - Camera images (if available)

Topics Published:
    /ai/sensor_batch - Batched sensor data
    /ai/stream_status - Streaming status

Parameters:
    batch_size (int): Number of samples per batch (default: 32)
    stream_rate_hz (float): Maximum streaming rate (default: 30.0)
    include_images (bool): Include camera images in stream (default: False)
    api_endpoint (str): External API endpoint (default: 'http://localhost:8000/ingest')
    buffer_size (int): Internal buffer size (default: 1000)

Author: Robot Book Chapter 2
License: Apache 2.0
"""

import json
import threading
from collections import deque
from dataclasses import dataclass, asdict
from typing import Optional, List, Dict, Any
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import JointState, Imu, Image
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor


@dataclass
class SensorSample:
    """Single timestamped sensor sample."""
    timestamp_ns: int
    joint_positions: List[float]
    joint_velocities: List[float]
    joint_names: List[str]
    imu_orientation: Optional[List[float]] = None
    imu_angular_velocity: Optional[List[float]] = None
    imu_linear_acceleration: Optional[List[float]] = None


@dataclass
class SensorBatch:
    """Batch of sensor samples for streaming."""
    batch_id: int
    start_time_ns: int
    end_time_ns: int
    sample_count: int
    samples: List[Dict[str, Any]]


class SensorStreamer(Node):
    """
    Stream sensor data to external AI training systems.

    This node enables the integration of simulation data with
    machine learning pipelines by:
    - Collecting sensor data at high frequency
    - Batching samples for efficient transfer
    - Streaming to external APIs (HTTP/gRPC)
    - Maintaining data integrity with timestamps

    Use this for:
    - Reinforcement Learning environments
    - Imitation Learning data collection
    - Behavior Cloning datasets
    - Real-time inference serving
    """

    def __init__(self):
        super().__init__('sensor_streamer')

        # ============================================
        # Parameters
        # ============================================

        self.declare_parameter(
            'batch_size', 32,
            ParameterDescriptor(description='Number of samples per batch')
        )
        self.declare_parameter(
            'stream_rate_hz', 30.0,
            ParameterDescriptor(description='Maximum streaming rate in Hz')
        )
        self.declare_parameter(
            'include_images', False,
            ParameterDescriptor(description='Include camera images in stream')
        )
        self.declare_parameter(
            'api_endpoint', 'http://localhost:8000/ingest',
            ParameterDescriptor(description='External API endpoint URL')
        )
        self.declare_parameter(
            'buffer_size', 1000,
            ParameterDescriptor(description='Internal buffer size')
        )
        self.declare_parameter(
            'enabled', True,
            ParameterDescriptor(description='Enable/disable streaming')
        )

        self.batch_size = self.get_parameter('batch_size').value
        self.stream_rate = self.get_parameter('stream_rate_hz').value
        self.include_images = self.get_parameter('include_images').value
        self.api_endpoint = self.get_parameter('api_endpoint').value
        buffer_size = self.get_parameter('buffer_size').value
        self.enabled = self.get_parameter('enabled').value

        # ============================================
        # State
        # ============================================

        self.buffer: deque = deque(maxlen=buffer_size)
        self.batch_id = 0
        self.total_samples = 0
        self.total_batches = 0
        self.dropped_samples = 0

        self.last_joint_state: Optional[JointState] = None
        self.last_imu: Optional[Imu] = None
        self.last_image: Optional[Image] = None

        self._lock = threading.Lock()

        # ============================================
        # QoS Profiles
        # ============================================

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cb_group = ReentrantCallbackGroup()

        # ============================================
        # Subscribers
        # ============================================

        self.joint_sub = self.create_subscription(
            JointState,
            '/sim/joint_states',
            self.joint_callback,
            sensor_qos,
            callback_group=self.cb_group
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/sim/imu/data',
            self.imu_callback,
            sensor_qos,
            callback_group=self.cb_group
        )

        if self.include_images:
            self.image_sub = self.create_subscription(
                Image,
                '/sim/camera/image_raw',
                self.image_callback,
                sensor_qos,
                callback_group=self.cb_group
            )

        # ============================================
        # Publishers
        # ============================================

        self.batch_pub = self.create_publisher(
            String,
            '/ai/sensor_batch',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/ai/stream_status',
            10
        )

        # ============================================
        # Timers
        # ============================================

        stream_period = 1.0 / self.stream_rate
        self.stream_timer = self.create_timer(
            stream_period,
            self.stream_batch,
            callback_group=self.cb_group
        )

        self.status_timer = self.create_timer(
            2.0,  # 0.5 Hz
            self.publish_status,
            callback_group=self.cb_group
        )

        # ============================================
        # Startup
        # ============================================

        self.get_logger().info(
            f'Sensor Streamer started: batch_size={self.batch_size}, '
            f'rate={self.stream_rate}Hz, endpoint={self.api_endpoint}'
        )

    # ============================================
    # Callbacks
    # ============================================

    def joint_callback(self, msg: JointState):
        """Handle incoming joint state messages."""
        if not self.enabled:
            return

        with self._lock:
            self.last_joint_state = msg
            self._add_sample()

    def imu_callback(self, msg: Imu):
        """Handle incoming IMU messages."""
        if not self.enabled:
            return

        with self._lock:
            self.last_imu = msg

    def image_callback(self, msg: Image):
        """Handle incoming camera images."""
        if not self.enabled:
            return

        with self._lock:
            self.last_image = msg

    # ============================================
    # Data Collection
    # ============================================

    def _add_sample(self):
        """Create and buffer a sensor sample."""
        if self.last_joint_state is None:
            return

        # Create sample from latest data
        sample = SensorSample(
            timestamp_ns=self.get_clock().now().nanoseconds,
            joint_positions=list(self.last_joint_state.position),
            joint_velocities=list(self.last_joint_state.velocity) if self.last_joint_state.velocity else [],
            joint_names=list(self.last_joint_state.name)
        )

        # Add IMU data if available
        if self.last_imu is not None:
            sample.imu_orientation = [
                self.last_imu.orientation.x,
                self.last_imu.orientation.y,
                self.last_imu.orientation.z,
                self.last_imu.orientation.w
            ]
            sample.imu_angular_velocity = [
                self.last_imu.angular_velocity.x,
                self.last_imu.angular_velocity.y,
                self.last_imu.angular_velocity.z
            ]
            sample.imu_linear_acceleration = [
                self.last_imu.linear_acceleration.x,
                self.last_imu.linear_acceleration.y,
                self.last_imu.linear_acceleration.z
            ]

        # Add to buffer
        if len(self.buffer) >= self.buffer.maxlen:
            self.dropped_samples += 1

        self.buffer.append(sample)
        self.total_samples += 1

    # ============================================
    # Streaming
    # ============================================

    def stream_batch(self):
        """Stream a batch of samples."""
        if not self.enabled:
            return

        with self._lock:
            if len(self.buffer) < self.batch_size:
                return

            # Extract batch
            samples = []
            for _ in range(min(self.batch_size, len(self.buffer))):
                sample = self.buffer.popleft()
                samples.append(asdict(sample))

        if not samples:
            return

        # Create batch
        batch = SensorBatch(
            batch_id=self.batch_id,
            start_time_ns=samples[0]['timestamp_ns'],
            end_time_ns=samples[-1]['timestamp_ns'],
            sample_count=len(samples),
            samples=samples
        )

        self.batch_id += 1
        self.total_batches += 1

        # Publish to ROS topic
        batch_msg = String()
        batch_msg.data = json.dumps(asdict(batch))
        self.batch_pub.publish(batch_msg)

        # Stream to external API (async in production)
        self._send_to_api(batch)

        self.get_logger().debug(f'Streamed batch {batch.batch_id} with {batch.sample_count} samples')

    def _send_to_api(self, batch: SensorBatch):
        """
        Send batch to external API.

        In production, this would use aiohttp or similar for async HTTP.
        For this example, we just log the action.
        """
        # Note: In production, implement actual HTTP/gRPC call
        # Example with requests:
        # try:
        #     response = requests.post(
        #         self.api_endpoint,
        #         json=asdict(batch),
        #         timeout=1.0
        #     )
        #     if response.status_code != 200:
        #         self.get_logger().warn(f'API returned {response.status_code}')
        # except Exception as e:
        #     self.get_logger().error(f'API call failed: {e}')
        pass

    # ============================================
    # Status
    # ============================================

    def publish_status(self):
        """Publish streaming status."""
        with self._lock:
            buffer_usage = len(self.buffer) / self.buffer.maxlen * 100

        status = {
            'enabled': self.enabled,
            'total_samples': self.total_samples,
            'total_batches': self.total_batches,
            'dropped_samples': self.dropped_samples,
            'buffer_usage_pct': round(buffer_usage, 1),
            'api_endpoint': self.api_endpoint
        }

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def set_enabled(self, enabled: bool):
        """Enable or disable streaming."""
        self.enabled = enabled
        state = 'enabled' if enabled else 'disabled'
        self.get_logger().info(f'Streaming {state}')

    def get_stats(self) -> dict:
        """Get streaming statistics."""
        with self._lock:
            return {
                'total_samples': self.total_samples,
                'total_batches': self.total_batches,
                'dropped_samples': self.dropped_samples,
                'buffer_size': len(self.buffer),
                'drop_rate': self.dropped_samples / max(1, self.total_samples) * 100
            }


def main(args=None):
    """Entry point for the sensor streamer."""
    rclpy.init(args=args)

    node = SensorStreamer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stats = node.get_stats()
        node.get_logger().info(f'Final stats: {stats}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
