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
gnome-terminal \
--tab --title "roscore" -e 'bash -c "sleep 1; roscore; exec bash;"' \
--tab --title "det2d" -e 'bash -c "rosrun image_tracking visual_tracking.py; exec bash;"' \
--tab --title "rearlight" -e 'bash -c "source ~/miniforge-pypy3/etc/profile.d/conda.sh; conda activate platoon; cd ../; rosrun rearlight_detection rearlight_detector.py; exec bash;"' \
--tab --title "fusion" -e 'bash -c "cd ../; rosrun fusion_detection fusion_detection; exec bash;"' \
--tab --title "urdf" -e 'bash -c "sleep 1; roslaunch --wait ../src/tracking/launch/urdf.launch; exec bash;"' \