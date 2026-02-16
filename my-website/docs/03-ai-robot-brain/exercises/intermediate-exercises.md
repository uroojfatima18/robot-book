---
id: chapter_3_intermediate_exercises
title: "Intermediate Tier Exercises"
sidebar_position: 41
chapter: chapter_3_ai_brain
section: exercises
---

# Intermediate Tier Exercises

**Test Your Skills in Perception, TF2, SLAM, and Navigation**

---

## Overview

These exercises test your practical skills from the Intermediate tier:
- I1: Camera and Depth Data Processing
- I2: TF2 Coordinate Frames
- I3: SLAM Toolbox Configuration
- I4: Nav2 Basics

Complete these exercises before moving to the Advanced tier.

---

## Exercise 1: Image Processing Pipeline

**Task**: Create a perception node that combines camera and depth data.

### Requirements

1. Subscribe to both `/camera/image_raw` and `/depth_camera/depth/image_raw`
2. Synchronize the two image streams (use `message_filters`)
3. For each synchronized pair:
   - Find the center pixel of the color image
   - Get the depth value at that pixel
   - Publish a `PointStamped` message with the 3D position

### Starter Code

```python
from message_filters import Subscriber, ApproximateTimeSynchronizer

# Create synchronized subscribers
rgb_sub = Subscriber(self, Image, '/camera/image_raw')
depth_sub = Subscriber(self, Image, '/depth_camera/depth/image_raw')

sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.1)
sync.registerCallback(self.synced_callback)

def synced_callback(self, rgb_msg, depth_msg):
    # Your code here
    pass
```

### Success Criteria
- [ ] Messages are synchronized within 100ms
- [ ] 3D point is published at 10+ Hz
- [ ] Depth values are in meters

---

## Exercise 2: Custom TF Broadcaster

**Task**: Create a TF broadcaster for a simulated pan-tilt camera.

### Requirements

1. Broadcast a `camera_pan` frame that rotates around Z axis
2. Broadcast a `camera_tilt` frame that rotates around Y axis
3. Accept pan and tilt angles as ROS 2 parameters
4. Update transforms at 50 Hz

### Frame Hierarchy

```
base_link
  └── camera_mount (static: 0, 0, 0.5)
        └── camera_pan (dynamic: rotates around Z)
              └── camera_tilt (dynamic: rotates around Y)
                    └── camera_optical (static: rotation only)
```

### Success Criteria
- [ ] Can set pan angle via parameter
- [ ] Can set tilt angle via parameter
- [ ] TF tree visible in RViz2
- [ ] Frames update smoothly

---

## Exercise 3: SLAM Parameter Tuning

**Task**: Optimize SLAM Toolbox for a specific environment.

### Scenario A: Long Corridor
- Environment: 50m long, 2m wide corridor
- Challenge: Limited features, potential drift

Recommended parameters to tune:
- `minimum_travel_distance`
- `max_laser_range`
- `resolution`

### Scenario B: Cluttered Office
- Environment: Many desks, chairs, and small obstacles
- Challenge: Dynamic objects, feature-rich but noisy

Recommended parameters to tune:
- `resolution`
- `scan_buffer_size`
- `loop_search_maximum_distance`

### Tasks
1. Create a parameter file for each scenario
2. Run SLAM in both environments
3. Compare map quality

### Success Criteria
- [ ] Corridor map has minimal drift
- [ ] Office map captures desk/chair layout
- [ ] Both maps are suitable for navigation

---

## Exercise 4: Waypoint Navigation

**Task**: Create a node that navigates through multiple waypoints.

### Requirements

1. Define a list of 4+ waypoints (x, y, yaw)
2. Navigate to each waypoint in sequence
3. Wait at each waypoint for 3 seconds
4. Handle navigation failures gracefully
5. Report progress (e.g., "Reached waypoint 2/4")

### Starter Code

```python
class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        self.waypoints = [
            (1.0, 0.0, 0.0),      # x, y, yaw
            (1.0, 1.0, 1.57),
            (0.0, 1.0, 3.14),
            (0.0, 0.0, -1.57),
        ]
        self.current_waypoint = 0

        # Your code here: set up action client

    def navigate_to_next_waypoint(self):
        # Your code here
        pass
```

### Success Criteria
- [ ] Robot visits all 4 waypoints
- [ ] Pauses 3 seconds at each
- [ ] Handles at least one navigation failure
- [ ] Returns to start position

---

## Exercise 5: Integrated Challenge

**Task**: Build a complete perception-to-navigation pipeline.

### Scenario

Your robot must:
1. Detect an orange cone in the camera image
2. Estimate the cone's 3D position using depth data
3. Transform the position to the map frame
4. Navigate to 0.5m in front of the cone
5. Stop and report success

### Components Needed

1. **Color detection** (I1): Find orange pixels
2. **Depth lookup** (I1): Get distance to cone
3. **TF transform** (I2): Convert camera→map frame
4. **Navigation goal** (I4): Send to Nav2

### Hints

```python
# Color detection (HSV range for orange)
lower_orange = np.array([10, 100, 100])
upper_orange = np.array([25, 255, 255])
mask = cv2.inRange(hsv_image, lower_orange, upper_orange)

# Find centroid of orange region
moments = cv2.moments(mask)
cx = int(moments['m10'] / moments['m00'])
cy = int(moments['m01'] / moments['m00'])
```

### Success Criteria
- [ ] Detects orange cone in various positions
- [ ] Accurately estimates 3D position (within 20cm)
- [ ] Transforms to map frame correctly
- [ ] Navigates to goal position
- [ ] Stops at appropriate distance

---

## Self-Assessment Checklist

Before moving to the Advanced tier, confirm you can:

- [ ] Use cv_bridge to process camera images
- [ ] Process depth images to find distances
- [ ] Create static and dynamic TF transforms
- [ ] Look up transforms between frames
- [ ] Configure and run SLAM Toolbox
- [ ] Save and load maps
- [ ] Launch Nav2 with a pre-built map
- [ ] Send navigation goals programmatically
- [ ] Monitor navigation status

---

## Debugging Tips

### Image Processing Issues
```bash
# Check if images are being published
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/image_raw --once | head -20
```

### TF Issues
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo base_link camera_link

# Monitor TF errors
ros2 run tf2_ros tf2_monitor
```

### Navigation Issues
```bash
# Check Nav2 node status
ros2 lifecycle list /planner_server
ros2 lifecycle list /controller_server

# Monitor navigation feedback
ros2 action list
ros2 topic echo /navigate_to_pose/_action/feedback
```

---

## Next Steps

**Congratulations on completing the Intermediate tier exercises!**

If you completed these successfully, you're ready for:
- [A1: Costmap Configuration](../advanced/A1-costmap-configuration.md)

If you struggled with any exercises, review the corresponding lesson before continuing.
