source ../../devel/setup.bash
chmod +x ../../src/rock/communication/v2v/src/socket_rock_client.py
echo "Run..."
gnome-terminal --window --title "rock_driver" -e 'bash -c "roslaunch --wait rock_driver rock_driver.launch ; exec bash;"'\
gnome-terminal --window --title "rock_comm" -e 'bash -c "roslaunch --wait rock_comm rock_comm.launch ; exec bash;"'