#!/bin/bash

source ../devel/setup.bash
cd ../
rosparam dump src/control/control/params/rock.yaml /rock_vehicle_node/
rosparam dump src/control/speed_controller/params/rock.yaml /speed_controller_test/
rosparam dump src/control/steer_controller/params/rock.yaml /pure_pursuit/
