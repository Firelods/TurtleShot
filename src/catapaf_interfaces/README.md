# Catapaf Interfaces

This package defines custom ROS 2 service interfaces used by the TurtleShot system.

## Overview

The `catapaf_interfaces` package provides service definitions for communication between behavior tree nodes and perception/navigation systems.

## Services

### GetDetection.srv

Request object detection for a specific label.

**Request:**
```
string label        # Object label to detect (e.g., "trash", "ball")
```

**Response:**
```
geometry_msgs/PoseStamped pose    # Detected object pose in map frame
bool is_detected                   # Whether the object was detected
```

**Usage:**
```bash
ros2 service call /get_detection catapaf_interfaces/srv/GetDetection "{label: 'trash'}"
```

### GetRandomGoal.srv

Generate a random valid navigation goal.

**Request:**
```
# No request fields
```

**Response:**
```
geometry_msgs/PoseStamped goal_pose    # Random goal pose
bool success                            # Whether a valid goal was generated
```

**Usage:**
```bash
ros2 service call /get_random_goal catapaf_interfaces/srv/GetRandomGoal
```

## Dependencies

- `geometry_msgs` - For `PoseStamped` message type
- `rosidl_default_generators` - For message generation

## Building

```bash
colcon build --packages-select catapaf_interfaces
source install/setup.bash
```

## Using in Other Packages

### Python
```python
from catapaf_interfaces.srv import GetDetection, GetRandomGoal

# Create client
client = node.create_client(GetDetection, 'get_detection')

# Create request
request = GetDetection.Request()
request.label = 'trash'

# Call service
future = client.call_async(request)
```

### C++
```cpp
#include "catapaf_interfaces/srv/get_detection.hpp"

auto client = node->create_client<catapaf_interfaces::srv::GetDetection>("get_detection");

auto request = std::make_shared<catapaf_interfaces::srv::GetDetection::Request>();
request->label = "trash";

auto future = client->async_send_request(request);
```

## License

Apache License 2.0
