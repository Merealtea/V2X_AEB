#!/bin/bash

source ../devel/setup.bash
rosparam set /use_sim_time false

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
--tab --title "visual_det" -e 'bash -c "source ~/miniforge-pypy3/etc/profile.d/conda.sh; conda activate Pytracking; echo $(which python); rosrun components_visual_tracking vehicle_tracking.py; exec bash;"' \