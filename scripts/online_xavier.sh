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
echo "Tracking..."
gnome-terminal \
--tab --title "tracking" -e 'bash -c "roslaunch --wait ../src/lidar_tracking/launch/tracking.launch; exec bash;"' \
--tab --title "efk_localization" -e 'bash -c "roslaunch --wait localization_pipeline localization_countryroad.launch; exec bash;"' \
--tab --title "control" -e 'bash -c "roslaunch --wait control vehicle.launch; exec bash;"' \
--tab --title "speed_controller" -e 'bash -c "roslaunch --wait speed_controller speed_controller_test.launch; exec bash;"' \
--tab --title "lqr_controller" -e 'bash -c "roslaunch --wait lqr LQR_controller.launch; exec bash;"' \
--tab --title "rviz" -e 'bash -c "roslaunch --wait control rviz.launch; exec bash;"' \
