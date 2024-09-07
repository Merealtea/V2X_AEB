source ../../devel/setup.bash

echo "Run..."
gnome-terminal \
--tab --title "center_comm" -e 'bash -c "roslaunch --wait center_comm center_comm.launch ; exec bash;"' 
gnome-terminal --window --title "center_fusion" -e 'bash -c "roslaunch --wait fusion center_fusion.launch ; exec bash;"'