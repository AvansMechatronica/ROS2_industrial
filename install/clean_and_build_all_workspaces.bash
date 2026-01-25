#!/bin/bash
set -e

# List of workspaces to source and build
workspaces=(
    "$HOME/pioneer_ws"
    "$HOME/xarm_ws"
    "$HOME/ros2_industrial_ws"
    # "$HOME/uros_ws"  # Uncomment if needed
    "$HOME/railtrack_ws"
    "$HOME/my_ur_ws"
    "$HOME/teachbot_ws"
    "$HOME/my_uf_ws"
)

for ws in "${workspaces[@]}"; do
    if [ -d "$ws" ]; then
        echo "Cleaning and building workspace: $ws"
        if [ -f "$ws/install/setup.bash" ]; then
            source "$ws/install/setup.bash"
        elif [ -f "$ws/install/local_setup.bash" ]; then
            source "$ws/install/local_setup.bash"
        fi
        cd "$ws"
        colcon clean || true
        colcon build --symlink-install
    else
        echo "Workspace not found: $ws"
    fi
    cd ~
    echo "--------------------------------------"
    sleep 1
done

echo "All workspaces cleaned and built!"
