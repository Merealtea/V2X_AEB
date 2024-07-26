#!/bin/bash

# rosbag record -a
# rosbag record /V2V/leader /driver/camera/image/compressed /radar_can_frame /radar_pc /rock_can/speed_feedback /rock_can/steer_feedback
rosbag record -e --lz4 \
    /driver/camera/image/compressed \
    /driver/sensing/image/compressed \
    /driver/fisheye/front/compressed \
    /radar_pc \
    /driver/hesai/pandar \
    /rock_can/.* \
    /Inertial/.* /wit/.* \
    /V2V/leader \
    /tf /tf_static \
    /tracking/front_vehicle/global_pose \
    /tracking/front_vehicle/local_pose \
    /tracking/front_vehicle/is_lost \
    /tracking/front_vehicle/speed \
    /tracking/components_tracking/rearlight \
    /lqr_debug  /control_debug \
    /lon_debug  /control/control_target \
    /mpc_debug   \
    /localization/estimation \
    -x /rock_can/can_write.*
