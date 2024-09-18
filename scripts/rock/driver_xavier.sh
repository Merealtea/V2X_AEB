#!/bin/bash

source ../devel/setup.bash
if [ -d "$HOME/anaconda3" ]; then  # on pc
    echo "conda environment activated:"
    source ~/anaconda3/etc/profile.d/conda.sh
    conda activate platoon_tracking
fi
if [ -d "$HOME/miniforge-pypy3" ]; then    # on Xavier
    echo "conda environment activated:"
    source ~/miniforge-pypy3/etc/profile.d/conda.sh
    conda activate platoon
fi
echo $(which python)
echo "Run drivers..."
# TODO: change fisheye camera driver
gnome-terminal \
--window --title "camera_driver" -e 'bash -c "cd ~/Desktop/camera_driver/hikrobot_ws; source devel/setup.bash; roslaunch --wait hikvision_camera run.launch"' \
--tab --title "radar_driver" -e 'bash -c "roslaunch --wait ../src/driver/microstar_can/radar_can_driver/launch/radar_can_driver.launch; exec bash;"' \
--tab --title "WIT_gps_driver" -e 'bash -c "roslaunch --wait ../src/driver/wit_ros_driver/launch/wit_gps.launch; exec bash;"' \
--tab --title "fisheye_driver" -e 'bash -c "roslaunch --wait ../src/driver/fisheye/drv/launch/drv_nodelet.launch; exec bash;"' \
# --tab --title "V2V_driver" -e 'bash -c "roslaunch --wait ../src/driver/v2v/launch/communication.launch; exec bash;"' \
