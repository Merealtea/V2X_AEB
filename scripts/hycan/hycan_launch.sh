source ../../devel/setup.bash
chmod +x ../../src/hycan/communication/src/socket_hycan_client.py
echo "Run..."
gnome-terminal --window --title "hycan_driver" -e 'bash -c "roslaunch --wait hycan_driver hycan_driver.launch ; exec bash;"'\
gnome-terminal --window --title "hycan_comm" -e 'bash -c "roslaunch --wait hycan_comm hycan_comm.launch ; exec bash;"'