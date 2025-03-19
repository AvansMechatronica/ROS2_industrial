#!/bin/bash

# Default device path
DEV_PATH="/dev/ttyUSB0"

# Check if an argument is provided
if [ $# -gt 0 ]; then
  DEV_PATH="$1"
fi

# Run the micro_ros_agent with the specified or default device
ros2 run micro_ros_agent micro_ros_agent serial --dev "$DEV_PATH"