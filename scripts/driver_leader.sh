#!/bin/bash

gnome-terminal \
--tab --title "roscore" -e 'bash -c "sleep 1; roscore; exec bash;"' \
--tab --title "leader_driver" -e 'bash -c "cd ~/Projects/Logistic_Net_Center; source devel/setup.bash; roslaunch --wait e100bringup driver.launch; exec bash;"' \
--tab --title "v2v_driver" -e 'bash -c "source ~/anaconda3/etc/profile.d/conda.sh; conda activate platoon; source ../devel/setup.bash; roslaunch --wait leader_v2v leader_publish.launch; exec bash;"' \
