#!/bin/bash
SCRIPT_DIR=$(cd $(dirname "$0") && pwd)
source /opt/ros/humble/setup.bash
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
else
    echo "Warning: install/setup.bash not found in $SCRIPT_DIR"
fi
ros2 launch "$SCRIPT_DIR/launch_sensors.py"
