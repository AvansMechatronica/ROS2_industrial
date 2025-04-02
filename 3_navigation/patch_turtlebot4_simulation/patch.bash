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
    echo "File exists"
else
    echo "File does not exist"
    mv $SPAWN_FILE $SPAWN_FILE$ORIGIN_EXT
    mv $SIM_FILE $SIM_FILE$ORIGIN_EXT
fi

cp $TURTLEBOT4_SPAWN_LAUNCH $TURTLEBOT4_SIM_DIR
cp $TURTLEBOT4_SIM_LAUNCH $TURTLEBOT4_SIM_DIR
