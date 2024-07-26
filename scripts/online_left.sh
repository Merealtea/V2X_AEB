#!/bin/bash

source ../devel/setup.bash

echo "Tracking..."
gnome-terminal \
--window --title "tracking" -e 'bash -c "roslaunch --wait ../src/lidar_tracking/launch/tracking.launch; exec bash;"' \
