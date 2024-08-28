source ../../devel/setup.bash
chmod +x ../../src/rock/communication/v2v/script/socket_rock_client.py
echo "Run..."
gnome-terminal --window --title "rock_comm" -e 'bash -c "roslaunch --wait rock_comm rock_comm.launch ; exec bash;"' \
gnome-terminal --window --title "rock_localization" -e 'bash -c "roslaunch --wait rock_localization localization.launch ; exec bash;"'