source ../../devel/setup.bash
chmod +x ../../src/rock/rock_communication/v2v/script/socket_rock_client.py
chmod +x ../../src/rock/rock_detection/src/detector.py
echo "Run..."
gnome-terminal --window --title "rock_comm" -e 'bash -c "roslaunch --wait rock_comm rock_comm.launch ; exec bash;"' \
gnome-terminal --window --title "rock_localization" -e 'bash -c "roslaunch --wait rock_localization localization.launch ; exec bash;"' \
gnome-terminal --window --title "rock_detection" -e 'bash -c "roslaunch --wait rock_detection detection.launch ; exec bash;"'