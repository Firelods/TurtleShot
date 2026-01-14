#!/bin/bash

# Script to wait for Nav2 action server to be available
# Usage: ./wait_for_nav2.sh [timeout_seconds]

TIMEOUT=${1:-120}  # Default 120 seconds timeout
ELAPSED=0
CHECK_INTERVAL=2

echo "Waiting for Nav2 'navigate_to_pose' action server to be available..."

while [ $ELAPSED -lt $TIMEOUT ]; do
    # Check if the action server is available
    if ros2 action list | grep -q "/navigate_to_pose"; then
        echo "✓ Nav2 action server is available!"
        
        # Additional check: verify bt_navigator node is running
        if ros2 node list | grep -q "bt_navigator"; then
            echo "✓ bt_navigator node is running!"
            exit 0
        else
            echo "  bt_navigator node not found yet, waiting..."
        fi
    fi
    
    sleep $CHECK_INTERVAL
    ELAPSED=$((ELAPSED + CHECK_INTERVAL))
    echo "  Still waiting... (${ELAPSED}s / ${TIMEOUT}s)"
done

echo "✗ Timeout waiting for Nav2 action server after ${TIMEOUT} seconds"
exit 1
