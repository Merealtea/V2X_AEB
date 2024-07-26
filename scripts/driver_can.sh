#!/bin/bash

source ../devel/setup.bash

echo "Run..."
gnome-terminal \
--window  --title "roscore" -e 'bash -c "sleep 1; roscore; exec bash;"' \
--tab --title "lkusbcan" -e 'bash -c "roslaunch --wait lkusbcan lcan_read_write.launch ; exec bash;"' \
--tab --title "can_decoder" -e 'bash -c "roslaunch --wait can_decoder can_decoder.launch; exec bash;"' \
--tab --title "can_encoder" -e 'bash -c "roslaunch --wait can_encoder can_encoder_auto_all.launch; exec bash;"' \
--tab --title "telecontroller" -e 'bash -c "roslaunch --wait ui control_ui.launch; exec bash;"' \
--tab --title "gps" -e 'bash -c "sleep 2; roslaunch oxford_gps_eth Inertial.launch; exec bash;"' \
--tab --title "calibration" -e 'bash -c "roslaunch --wait calibration cyberRock_tf.launch; exec bash;"' \
--tab --title "V2V_driver" -e 'bash -c "roslaunch --wait v2v communication.launch; exec bash;"' \
