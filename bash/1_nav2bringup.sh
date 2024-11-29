#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/ros2_ws
# export ROS_DOMAIN_ID=57
while : ;do
    read -p "Press [Enter] key to start tb3_simulation..."
    source install/setup.bash
    ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True params_file:=/home/yakifrog/ros2_ws/params/nav2_params.yaml
done