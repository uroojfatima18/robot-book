#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Physical AI & Humanoid Robotics Textbook
#
# Chapter 1: The Robotic Nervous System (ROS 2)
# Lesson: I1 - Nodes, Topics, Services, and Actions
# Example: Simple Service Server and Client

"""
ROS 2 Service Example - Add Two Integers

This module demonstrates the service pattern in ROS 2:
1. Service Server: Waits for requests and returns responses
2. Service Client: Sends requests and receives responses

The service adds two integers and returns their sum.

Usage:
    # Terminal 1: Run the server
    python3 simple_service.py server

    # Terminal 2: Run the client
    python3 simple_service.py client 5 3

    # Or use CLI to call the service
    ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    """
    Service server that adds two integers.

    Creates a service at /add_two_ints that accepts two integers
    and returns their sum.
    """

    def __init__(self):
        """Initialize the service server."""
        super().__init__('add_two_ints_server')

        # Create the service
        # Arguments: service_type, service_name, callback
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback
        )

        self.get_logger().info('Service server ready: /add_two_ints')
        self.get_logger().info('Waiting for requests...')

    def add_callback(self, request, response):
        """
        Service callback - processes incoming requests.

        Args:
            request: AddTwoInts.Request with fields 'a' and 'b'
            response: AddTwoInts.Response with field 'sum'

        Returns:
            The populated response object
        """
        response.sum = request.a + request.b

        self.get_logger().info(
            f'Request received: {request.a} + {request.b} = {response.sum}'
        )

        return response


class AddTwoIntsClient(Node):
    """
    Service client that requests addition of two integers.

    Connects to the /add_two_ints service, sends a request,
    and waits for the response.
    """

    def __init__(self):
        """Initialize the service client."""
        super().__init__('add_two_ints_client')

        # Create the client
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        self.get_logger().info('Waiting for service to be available...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting...')

        self.get_logger().info('Service is available!')

    def send_request(self, a: int, b: int) -> int:
        """
        Send a request to the service and wait for response.

        Args:
            a: First integer
            b: Second integer

        Returns:
            The sum of a and b
        """
        # Create request
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        self.get_logger().info(f'Sending request: {a} + {b}')

        # Send request asynchronously
        future = self.client.call_async(request)

        # Wait for the response
        rclpy.spin_until_future_complete(self, future)

        # Get the result
        result = future.result()
        return result.sum


def run_server():
    """Run the service server."""
    rclpy.init()
    server = AddTwoIntsServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('Server shutting down...')
    finally:
        server.destroy_node()
        rclpy.shutdown()


def run_client(a: int, b: int):
    """Run the service client with given arguments."""
    rclpy.init()
    client = AddTwoIntsClient()

    try:
        result = client.send_request(a, b)
        client.get_logger().info(f'Result: {a} + {b} = {result}')
    except Exception as e:
        client.get_logger().error(f'Service call failed: {e}')
    finally:
        client.destroy_node()
        rclpy.shutdown()


def main():
    """
    Main entry point.

    Usage:
        python3 simple_service.py server       # Run the server
        python3 simple_service.py client 5 3   # Run client with args
    """
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 simple_service.py server")
        print("  python3 simple_service.py client <a> <b>")
        sys.exit(1)

    mode = sys.argv[1].lower()

    if mode == 'server':
        run_server()
    elif mode == 'client':
        if len(sys.argv) < 4:
            print("Error: client mode requires two integer arguments")
            print("  python3 simple_service.py client <a> <b>")
            sys.exit(1)
        try:
            a = int(sys.argv[2])
            b = int(sys.argv[3])
            run_client(a, b)
        except ValueError:
            print("Error: arguments must be integers")
            sys.exit(1)
    else:
        print(f"Unknown mode: {mode}")
        print("Use 'server' or 'client'")
        sys.exit(1)


if __name__ == '__main__':
    main()
