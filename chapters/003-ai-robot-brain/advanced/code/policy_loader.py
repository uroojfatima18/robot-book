#!/usr/bin/env python3
"""
policy_loader.py

Loads a pre-trained ONNX policy and runs inference.
Demonstrates sim-to-real policy deployment pattern.

Dependencies: onnxruntime, numpy

Usage:
  python3 policy_loader.py

Example with ROS 2:
  # In your ROS 2 node
  from policy_loader import PolicyLoader
  policy = PolicyLoader('path/to/policy.onnx')
  action = policy.get_action(observation)
"""

import numpy as np

try:
    import onnxruntime as ort
except ImportError:
    print("Install onnxruntime: pip install onnxruntime")
    raise


class PolicyLoader:
    """Loads and runs inference on an ONNX policy."""

    def __init__(self, model_path: str, use_gpu: bool = False):
        """Initialize the policy loader.

        Args:
            model_path: Path to .onnx policy file
            use_gpu: Whether to use GPU for inference (requires onnxruntime-gpu)
        """
        # Select execution provider
        if use_gpu:
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        else:
            providers = ['CPUExecutionProvider']

        # Create inference session
        self.session = ort.InferenceSession(model_path, providers=providers)

        # Get input/output info
        self.input_name = self.session.get_inputs()[0].name
        self.input_shape = self.session.get_inputs()[0].shape
        self.output_name = self.session.get_outputs()[0].name
        self.output_shape = self.session.get_outputs()[0].shape

        print(f"Loaded policy: {model_path}")
        print(f"Input: {self.input_name} {self.input_shape}")
        print(f"Output: {self.output_name} {self.output_shape}")
        print(f"Provider: {self.session.get_providers()[0]}")

    def get_action(self, observation: np.ndarray) -> np.ndarray:
        """Run policy inference to get action.

        Args:
            observation: Current state observation (1D numpy array)

        Returns:
            Action to execute (1D numpy array)
        """
        # Ensure correct shape and type
        obs = observation.astype(np.float32)
        if len(obs.shape) == 1:
            obs = obs.reshape(1, -1)  # Add batch dimension

        # Run inference
        outputs = self.session.run(
            [self.output_name],
            {self.input_name: obs}
        )

        action = outputs[0][0]  # Remove batch dimension
        return action

    def get_action_batch(self, observations: np.ndarray) -> np.ndarray:
        """Run policy inference on a batch of observations.

        Args:
            observations: Batch of observations (2D numpy array: [batch, obs_dim])

        Returns:
            Actions for each observation (2D numpy array: [batch, action_dim])
        """
        obs = observations.astype(np.float32)
        outputs = self.session.run(
            [self.output_name],
            {self.input_name: obs}
        )
        return outputs[0]


class SafePolicyExecutor:
    """Executes a policy with safety limits and monitoring."""

    def __init__(self, policy: PolicyLoader, config: dict = None):
        """Initialize safe executor.

        Args:
            policy: Loaded policy
            config: Safety configuration dict
        """
        self.policy = policy

        # Default safety config
        default_config = {
            'max_action': 1.0,
            'action_scale': 1.0,
            'action_offset': 0.0,
        }
        self.config = {**default_config, **(config or {})}

    def get_safe_action(self, observation: np.ndarray) -> np.ndarray:
        """Get action with safety limits applied.

        Args:
            observation: Current state observation

        Returns:
            Clipped and scaled action
        """
        # Get raw action from policy
        raw_action = self.policy.get_action(observation)

        # Apply scaling and offset
        action = raw_action * self.config['action_scale'] + self.config['action_offset']

        # Clip to safety limits
        action = np.clip(action, -self.config['max_action'], self.config['max_action'])

        return action


def main():
    """Example usage with dummy observations."""

    print("=" * 50)
    print("Policy Loader Demo")
    print("=" * 50)

    # In a real scenario, load from file
    # policy = PolicyLoader('pretrained/locomotion_policy.onnx')

    # For demo, create a dummy session
    print("\nNote: This demo shows the API structure.")
    print("In practice, load a real ONNX policy file.")
    print("\nExample usage:")
    print("  policy = PolicyLoader('path/to/policy.onnx')")
    print("  action = policy.get_action(observation)")

    # Simulate the expected observation format for a quadruped
    print("\nTypical observation for quadruped robot:")
    obs_components = [
        ("Base angular velocity (3)", 3),
        ("Projected gravity (3)", 3),
        ("Commands (3: vx, vy, yaw)", 3),
        ("Joint positions (12)", 12),
        ("Joint velocities (12)", 12),
        ("Previous actions (12)", 12),
    ]

    total_dim = 0
    for name, dim in obs_components:
        print(f"  {name}: {dim} dims")
        total_dim += dim
    print(f"  Total: {total_dim} dimensions")

    print("\nTypical action for quadruped robot:")
    print("  Target joint positions: 12 dimensions")

    print("\n" + "=" * 50)
    print("Safety Considerations for Deployment:")
    print("=" * 50)
    print("1. Always test at reduced speed first")
    print("2. Have emergency stop accessible")
    print("3. Monitor joint velocities and torques")
    print("4. Use SafePolicyExecutor for action clipping")
    print("5. Implement hardware-level safety limits")


if __name__ == '__main__':
    main()
