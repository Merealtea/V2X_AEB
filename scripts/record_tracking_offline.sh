#!/bin/bash

source ../devel/setup.bash
rosparam set /use_sim_time true

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

# gnome-terminal --tab --title "vehicle_mapping" -e 'bash -c "roslaunch --wait vehicle_mapping vehicle_mapping.launch; exec bash;"'

gnome-terminal \
--tab --title "roscore" -e 'bash -c "roscore; exec bash;"' \
--tab --title "calib" -e 'bash -c "roslaunch --wait calibration cyberRock_tf.launch; exec bash;"' \
--tab --title "vehicle_mapping" -e 'bash -c "roslaunch --wait vehicle_mapping vehicle_mapping.launch; exec bash;"' \
--tab --title "visual_det" -e 'bash -c "source ~/anaconda3/etc/profile.d/conda.sh; conda activate platoon_tracking; echo $(which python); rosrun image_tracking vehicle_detection.py; exec bash;"' \
--tab --title "visual_det" -e 'bash -c "source ~/anaconda3/etc/profile.d/conda.sh; conda activate Pytracking; echo $(which python); rosrun components_visual_tracking components_tracking.py; exec bash;"' \
--tab --title "components_tracking" -e 'bash -c "roslaunch --wait components_visual_tracking components_tracking.launch; exec bash;"' \
--tab --title "tracking" -e 'bash -c "roslaunch --wait ../src/tracking/launch/tracking.launch; exec bash;"' \
--tab --title "tracking_visualization" -e 'bash -c "rviz -d ../src/tracking/rviz/tracking_record.rviz; exec bash;"' \
