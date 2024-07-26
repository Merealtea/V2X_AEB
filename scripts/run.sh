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
--tab --title "roscore" -e 'bash -c "sleep 1; roscore; exec bash;"' \
--tab --title "tf_publisher" -e 'bash -c "roslaunch --wait calibration cyberRock_tf.launch; exec bash;"' \
--tab --title "radar_driver" -e 'bash -c "roslaunch --wait ../src/driver/microstar_can/radar_can_driver/launch/radar_can_driver.launch; exec bash;"' \
--tab --title "camera_driver" -e 'bash -c "cd ~/Desktop/camera_driver/hikrobot_ws; source devel/setup.bash; roslaunch --wait hikvision_camera run.launch"' \
--tab --title "V2V_driver" -e 'bash -c "roslaunch --wait ../src/driver/v2v/launch/communication.launch; exec bash;"' \
--tab --title "WIT_gps_driver" -e 'bash -c "roslaunch --wait ../src/driver/wit_ros_driver/launch/wit_gps.launch; exec bash;"' \
--tab --title "tracking" -e 'bash -c "roslaunch --wait ../src/tracking/launch/tracking.launch; exec bash;"' \
--tab --title "efk_localization" -e 'bash -c "roslaunch --wait localization_pipeline localization_countryroad.launch; exec bash;"' \
--tab --title "control" -e 'bash -c "roslaunch --wait control vehicle.launch; exec bash;"' \
--tab --title "speed_controller" -e 'bash -c "roslaunch --wait speed_controller speed_controller_test.launch; exec bash;"' \
--tab --title "steer_controller" -e 'bash -c "roslaunch --wait steer_controller pure_pursuit.launch; exec bash;"' \
--tab --title "vehicle_display" -e 'bash -c "roslaunch --wait control urdf.launch; exec bash;"' \
--tab --title "rviz" -e 'bash -c "roslaunch --wait control rviz.launch; exec bash;"' \
--tab --title "rqt_multiplot" -e 'bash -c "rqt_multiplot --multiplot-config ../src/control/control/rviz/rqt_multiplot.xml; exec bash;"' \
