# Pre-trained Policies

This directory contains pre-trained ONNX policy files for sim-to-real deployment.

## Expected Files

### locomotion_policy.onnx

A pre-trained locomotion policy for a quadruped robot (e.g., ANYmal, Unitree Go1).

**Input (observation):**
- Shape: `[1, 48]` (float32)
- Contents:
  - Base angular velocity: 3 dims
  - Projected gravity: 3 dims
  - Commands (vx, vy, yaw_rate): 3 dims
  - Joint positions: 12 dims
  - Joint velocities: 12 dims
  - Previous actions: 12 dims

**Output (action):**
- Shape: `[1, 12]` (float32)
- Contents: Target joint positions for 12 joints

## Training Source

Pre-trained policies can be obtained from:

1. **NVIDIA Isaac Gym Examples**
   - https://github.com/NVIDIA-Omniverse/IsaacGymEnvs
   - Train using `python train.py task=Anymal`
   - Export using `python export_onnx.py`

2. **legged_gym**
   - https://github.com/leggedrobotics/legged_gym
   - Contains policies for various quadrupeds

3. **Your Own Training**
   - Train in Isaac Gym or similar
   - Export to ONNX format

## Exporting PyTorch to ONNX

```python
import torch

# Load your trained policy
policy = torch.load('trained_policy.pt')
policy.eval()

# Create dummy input
dummy_input = torch.randn(1, 48)

# Export to ONNX
torch.onnx.export(
    policy,
    dummy_input,
    'locomotion_policy.onnx',
    input_names=['observation'],
    output_names=['action'],
    dynamic_axes={
        'observation': {0: 'batch_size'},
        'action': {0: 'batch_size'}
    }
)
```

## Verification

```python
import onnxruntime as ort
import numpy as np

# Load and test
session = ort.InferenceSession('locomotion_policy.onnx')

# Test inference
obs = np.random.randn(1, 48).astype(np.float32)
action = session.run(None, {'observation': obs})[0]

print(f"Input shape: {obs.shape}")
print(f"Output shape: {action.shape}")
```

## Safety Notes

> **WARNING**: Pre-trained policies may not work directly on your robot without additional tuning.

Before deployment:
1. Verify observation/action dimensions match your robot
2. Test at reduced speed in a safe environment
3. Have emergency stop ready
4. Monitor joint limits and velocities
