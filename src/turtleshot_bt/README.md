# TurtleShot Behavior Tree Orchestrator

## Overview

**turtleshot_bt** is a professional ROS2 package implementing a Behavior Tree-based orchestrator for the TurtleShot robot using **BehaviorTree.CPP** and **Groot2**.

This is the **correct** way to implement task planning in ROS2, as described in:
- [BehaviorTree.CPP ROS2 Integration](https://www.behaviortree.dev/docs/ros2_integration/)
- Nav2 Behavior Tree Navigator
- Your course slides 45-57 on Task Planning

## Key Features

✅ **C++ Implementation** with BehaviorTree.CPP v3
✅ **ROS2 Native** integration via behaviortree_ros2
✅ **Groot2 Live Monitoring** via ZMQ (ports 1666/1667)
✅ **Visual Editing** - Edit XML in Groot2, reload at runtime
✅ **Nav2 Integration** - NavigateToPose action client
✅ **Modular Architecture** - Separate leaf nodes for each capability

## Architecture

```
turtleshot_bt (C++ Orchestrator)
       │
       ├─ Calls Actions/Services from other packages:
       │    ├─ Nav2 (navigation)
       │    ├─ catapaf_gazebo (catapult control)
       │    └─ video_to_ai (vision detection)
       │
       └─ Monitored in real-time by Groot2 via ZMQ
```

### BT Leaf Nodes

| Node Type | Name | Description |
|-----------|------|-------------|
| **Action** | NavigateToPose | Calls Nav2 to navigate to (x, y, yaw) |
| **Action** | FireCatapult | Triggers catapult firing via service |
| **Condition** | HasTarget | Checks if vision detected target |
| **Condition** | TargetInRange | Checks if target is within range |

## Installation

### 1. Install Dependencies

```bash
sudo apt install ros-${ROS_DISTRO}-behaviortree-cpp-v3 \
                 ros-${ROS_DISTRO}-behaviortree-ros2 \
                 ros-${ROS_DISTRO}-nav2-msgs
```

### 2. Build Package

```bash
cd ~/TurtleShot  # Your workspace
colcon build --packages-select turtleshot_bt
source install/setup.bash
```

### 3. Install Groot2

Download from: https://www.behaviortree.dev/groot/

Or build from source:
```bash
git clone https://github.com/BehaviorTree/Groot.git
cd Groot
# Follow build instructions
```

## Usage

### Quick Start

```bash
# Terminal 1: Launch everything (sim + BT orchestrator)
ros2 launch turtleshot_bt turtleshot_bringup.launch.py

# Terminal 2: Open Groot2 for live monitoring
# In Groot2: Monitor → Connect to localhost:1666
```

### Launch Options

```bash
# Without Gazebo simulation
ros2 launch turtleshot_bt turtleshot_bringup.launch.py sim:=false

# With Nav2 navigation stack
ros2 launch turtleshot_bt turtleshot_bringup.launch.py nav:=true

# BT orchestrator only
ros2 run turtleshot_bt turtleshot_bt_node
```

## Editing the Behavior Tree

### Method 1: Edit XML Directly

Edit `trees/turtleshot_mission.xml`:

```xml
<Sequence name="MainSequence">
  <Action ID="NavigateToPose" x="1.0" y="0.5" yaw="0.0"/>
  <Action ID="FireCatapult"/>
</Sequence>
```

Then restart the BT node.

### Method 2: Edit in Groot2 (Recommended)

1. **Open Groot2**
2. **File → Load Tree**
3. Navigate to: `install/turtleshot_bt/share/turtleshot_bt/trees/turtleshot_mission.xml`
4. **Edit graphically**:
   - Drag nodes from palette
   - Modify parameters in property panel
   - Rearrange tree structure
5. **File → Save Tree**
6. Restart BT orchestrator

Changes take effect immediately on next launch!

## Live Monitoring with Groot2

While the BT is running:

1. **Launch Groot2**
2. **Monitor → Connect**
3. Default settings (localhost:1666/1667) - Click Connect
4. Watch the tree execute in real-time:
   - 🟢 Green = SUCCESS
   - 🟡 Yellow = RUNNING
   - 🔴 Red = FAILURE
   - ⚪ Gray = IDLE

This is **exactly** what's shown in the BehaviorTree.ROS2 docs and Turtlesim_BT example!

## Mission Flow

The default mission (`turtleshot_mission.xml`):

1. **Find Person with Ball**
   → Retry until vision detects person

2. **Navigate to Person**
   → Use Nav2 to approach

3. **Wait for Person in Range**
   → Check LiDAR distance < 0.5m

4. **Find Trash Bin**
   → Retry until vision detects trash

5. **Navigate to Trash Bin**
   → Use Nav2 to approach

6. **Fire Catapult**
   → Call service to shoot ball

## Adding Custom Nodes

### 1. Create Node Class

```cpp
// include/turtleshot_bt/actions/my_action.hpp
#include <behaviortree_ros2/bt_action_node.hpp>

class MyAction : public BT::RosActionNode<MyActionType>
{
public:
  MyAction(const std::string & name,
           const BT::NodeConfiguration & config,
           const BT::RosNodeParams & params);

  static BT::PortsList providedPorts() { /* ... */ }
  bool setGoal(Goal & goal) override { /* ... */ }
  BT::NodeStatus onResultReceived(const WrappedResult & result) override { /* ... */ }
};
```

### 2. Implement in .cpp

```cpp
// src/actions/my_action.cpp
#include "turtleshot_bt/actions/my_action.hpp"
// Implementation...
```

### 3. Register in main node

```cpp
// src/turtleshot_bt_node.cpp
factory.registerNodeType<MyAction>("MyAction", params);
```

### 4. Use in XML

```xml
<Action ID="MyAction" param1="value"/>
```

### 5. Update CMakeLists.txt

```cmake
add_executable(turtleshot_bt_node
  src/turtleshot_bt_node.cpp
  src/actions/my_action.cpp  # Add this
  # ...
)
```

## Integration with Existing Packages

### Nav2

Already integrated! `NavigateToPose` action calls `/navigate_to_pose`.

Ensure Nav2 is running:
```bash
ros2 launch nav2_bringup bringup_launch.py
```

### Catapult Control

`FireCatapult` expects a service at `/fire_catapult` (std_srvs/Trigger).

Add to catapaf_gazebo:
```python
self.fire_service = self.create_service(
    Trigger,
    '/fire_catapult',
    self.fire_callback)
```

### Vision AI

BT conditions subscribe to `/detections` (vision_msgs/Detection2DArray).

Ensure your vision node publishes to this topic.

## Troubleshooting

### Build Errors

**Missing behaviortree_cpp_v3:**
```bash
sudo apt install ros-${ROS_DISTRO}-behaviortree-cpp-v3
```

**Missing nav2_msgs:**
```bash
sudo apt install ros-${ROS_DISTRO}-nav2-msgs
```

### Runtime Issues

**"Failed to load behavior tree"**
- Check XML syntax
- Ensure tree file exists in install directory
- Verify all node IDs match registered types

**"Action server not available"**
- Nav2 not running → Launch nav2_bringup
- Wrong action name → Check `params.default_port_value`

**Groot2 can't connect**
- Check ports 1666/1667 not in use
- Verify `PublisherZMQ` is created in code
- Try `localhost` instead of `127.0.0.1`

### ZMQ Monitoring Not Working

Ensure ZMQ publisher is created:
```cpp
BT::PublisherZMQ publisher_zmq(tree, 10, 1666, 1667);
```

In Groot2:
- Monitor → Connect
- Publisher port: 1666
- Server port: 1667

## File Structure

```
turtleshot_bt/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/
│   └── turtleshot_bt/
│       ├── actions/
│       │   ├── navigate_to_pose_action.hpp
│       │   └── fire_catapult_action.hpp
│       └── conditions/
│           ├── has_target_condition.hpp
│           └── target_in_range_condition.hpp
├── src/
│   ├── turtleshot_bt_node.cpp          ← Main entry point
│   ├── actions/
│   │   ├── navigate_to_pose_action.cpp
│   │   └── fire_catapult_action.cpp
│   └── conditions/
│       ├── has_target_condition.cpp
│       └── target_in_range_condition.cpp
├── trees/
│   └── turtleshot_mission.xml          ← BT definition (edit in Groot2!)
└── launch/
    └── turtleshot_bringup.launch.py
```

## Comparison: Python vs C++ BT

| Feature | Python (catapaf_gazebo) | C++ (turtleshot_bt) |
|---------|-------------------------|---------------------|
| Library | Custom implementation | BehaviorTree.CPP |
| Groot2 support | ❌ No live monitoring | ✅ Full ZMQ integration |
| Nav2 integration | ⚠️ Manual | ✅ Native RosActionNode |
| Performance | Slower | Faster |
| Ecosystem | Limited | Nav2, MoveIt, official docs |
| **Recommendation** | Prototype/learning | **Production use** |

## Next Steps

1. **Configure Nav2** for your robot
   - Create navigation params
   - Generate map with SLAM
   - Test NavigateToPose separately

2. **Add more BT nodes**
   - PickupObject action
   - ObstacleAvoidance behavior
   - BatteryMonitor condition

3. **Create mission variants**
   - Save different XML files
   - Switch between missions at launch

4. **Integrate with py_trees_ros** (optional)
   - If you need Python-based behaviors
   - Can mix C++ and Python nodes

## References

- [BehaviorTree.CPP Docs](https://www.behaviortree.dev/)
- [BehaviorTree.ROS2 Integration](https://www.behaviortree.dev/docs/ros2_integration/)
- [Groot2](https://www.behaviortree.dev/groot/)
- [Nav2 BT Navigator](https://navigation.ros.org/behavior_trees/index.html)
- [Example: Turtlesim_BT](https://github.com/sherif1152/Turtlesim_BT)

## License

Apache 2.0

## Maintainer

Clément - clement@example.com
