source ../../devel/setup.bash

echo "Run..."
gnome-terminal \
--tab --title "center_comm" -e 'bash -c "roslaunch --wait center_comm center_comm.launch ; exec bash;"' 