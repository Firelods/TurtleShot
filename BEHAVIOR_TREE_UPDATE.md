# Behavior Tree Update Summary

## Overview
Updated the robot's behavior tree algorithm to implement a new workflow sequence.

## New Workflow Sequence

The robot now follows these steps:

1. **Go to Human** - Detect and navigate to a human with a ball
2. **Stop** - Stop the robot upon reaching the human
3. **Wait 30 seconds** - Wait for 30 seconds near the human
4. **Step Back** - Move backward 0.5 meters
5. **Random Walk** - Perform random exploration while searching for trash
6. **Find Trash** - Continuously check for trash can detection during random walk
7. **Go to Trash** - Navigate to the detected trash can
8. **Trigger Arm** - Activate the robot's arm mechanism

## Changes Made

### 1. Behavior Tree XML (`patrol_and_interact.xml`)

**Key Changes:**
- Restructured the main sequence to follow the new workflow
- Added `StepBack` action with 0.5m distance parameter
- Implemented `ReactiveFallback` pattern for concurrent random walk and trash detection
- Added proper action definitions in TreeNodesModel for Groot2 visualization

**How Random Walk + Trash Detection Works:**
The `ReactiveFallback` node continuously:
- First checks if trash is detected (if SUCCESS, exits and proceeds)
- If trash not detected (FAILURE), performs RandomWalk (returns RUNNING)
- This creates a "search while exploring" behavior

### 2. C++ Implementation (`bt_main.cpp`)

**New StepBack Action:**
- Created `StepBack` class as a `StatefulActionNode`
- Uses TF transforms to track distance traveled
- Moves robot backward at 0.2 m/s
- Configurable distance parameter (default: 0.5m)
- Stops automatically when target distance is reached

**Registration:**
- Registered `StepBack` action in the BehaviorTreeFactory
- Added proper builder lambda for ROS node integration

## Behavior Flow Diagram

```
Start
  ↓
[Detect Human] (retry until found)
  ↓
[Navigate to Human]
  ↓
[Stop Robot]
  ↓
[Wait 30 seconds]
  ↓
[Step Back 0.5m]
  ↓
[Random Walk + Detect Trash] ← loops until trash found
  ↓
[Stop Robot]
  ↓
[Navigate to Trash]
  ↓
[Trigger Arm]
  ↓
End
```

## Technical Details

### StepBack Implementation
- **Type:** StatefulActionNode
- **Input Port:** `distance` (double, default: 0.5m)
- **Method:** Uses odometry via TF transforms (map → base_link)
- **Speed:** -0.2 m/s (backward)
- **Completion:** Euclidean distance check from start position

### ReactiveFallback Pattern
- Ticks all children every cycle (reactive)
- Returns SUCCESS if any child succeeds
- Returns RUNNING if all children return RUNNING
- Returns FAILURE only if all children fail

This ensures the robot continuously searches for trash while exploring randomly.

## Files Modified

1. `/home/bilbo/TurtleShot/src/catapaf_gazebo/behavior_trees/patrol_and_interact.xml`
2. `/home/bilbo/TurtleShot/src/catapaf_bt/src/bt_main.cpp`

## Build Status

✅ Successfully compiled with no errors
⚠️ Minor warnings (unused parameters) - safe to ignore

## Next Steps

To test the updated behavior:

1. Source the workspace:
   ```bash
   source install/setup.bash
   ```

2. Launch the system (use your existing launch script)

3. The robot will:
   - Search for a human with a ball
   - Approach and wait 30 seconds
   - Step back 0.5m
   - Explore randomly while looking for trash
   - Navigate to trash when found
   - Trigger the arm

## Notes

- The 30-second wait duration can be adjusted in the XML file (line 15)
- The step-back distance can be modified in the XML file (line 18)
- Random walk behavior uses the existing obstacle avoidance implementation
- All actions use proper ROS2 integration with TF, cmd_vel, and service calls
