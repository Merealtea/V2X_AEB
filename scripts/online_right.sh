#!/bin/bash

source ../devel/setup.bash

echo "Control ..."
gnome-terminal \
--window --title "efk_localization" -e 'bash -c "roslaunch --wait localization_pipeline localization_countryroad.launch; exec bash;"' \
--tab --title "track_manager" -e 'bash -c "roslaunch --wait track_manager track_manager.launch; exec bash;"' \
--tab --title "rviz" -e 'bash -c "roslaunch --wait track_manager rviz.launch; exec bash;"' \
# --tab --title "lon_controller" -e 'bash -c "roslaunch --wait longitudinal_controller lon_controller.launch; exec bash;"' \
# --tab --title "lqr_controller" -e 'bash -c "roslaunch --wait lqr LQR_controller.launch; exec bash;"' \
#--tab --title "speed_controller" -e 'bash -c "roslaunch --wait speed_controller speed_controller_test.launch; exec bash;"' \
