#!/bin/bash

source ~/Desktop/CyberRock_IVFC/devel/setup.bash

echo "run..."
gnome-terminal --window --title "pandar" -e 'bash -c "roslaunch --wait hesai_lidar p40.launch; exec bash;"' \
