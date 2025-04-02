#!/bin/bash

USERNAME=$(whoami)
TURTLEBOT4_SIM_DIR="/home/"$USERNAME"/turtlebot_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/launch"
TURTLEBOT4_SPAWN_LAUNCH="turtlebot4_spawn.launch.py"
TURTLEBOT4_SIM_LAUNCH="sim.launch.py"

SPAWN_FILE=$TURTLEBOT4_SIM_DIR"/"$TURTLEBOT4_SPAWN_LAUNCH
SIM_FILE=$TURTLEBOT4_SIM_DIR"/"$TURTLEBOT4_SIM_LAUNCH
ORIGIN_EXT=".org"

echo $SPAWN_FILE
if [ -f $SPAWN_FILE$ORIGIN_EXT ]; then
    cp $SPAWN_FILE$ORIGIN_EXT $SPAWN_FILE 
    cp $SIM_FILE$ORIGIN_EXT $SIM_FILE 
    echo "File exists"
else
    echo "File does not exist"
fi