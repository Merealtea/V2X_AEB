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

# gnome-terminal --tab --title "vehicle_mapping" -e 'bash -c "roslaunch --wait vehicle_mapping vehicle_mapping.launch; exec bash;"'

gnome-terminal \
--tab --title "roscore" -e 'bash -c "roscore; exec bash;"' \
--tab --title "calib" -e 'bash -c "roslaunch --wait calibration cyberRock_tf.launch; exec bash;"' \
--tab --title "vis" -e 'bash -c "roslaunch --wait exp_pose_vis_compare exp_pose_vis_compare.launch; exec bash;"' \
--tab --title "rviz" -e 'bash -c "rviz -d ../src/utils/exp_pose_vis_compare/rviz/compare.rviz; exec bash;"' \