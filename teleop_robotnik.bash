#!/bin/sh

export NETAPP_ADDRESS=http://192.168.60.211:31012
export USE_MIDDLEWARE=false
#export MIDDLEWARE_ADDRESS=crop.5gera.net
export MIDDLEWARE_ADDRESS=middleware.5gera.robotnik.cloud
export MIDDLEWARE_USER=ad20f254-dc3b-406d-9f15-b73ccd47e867
export MIDDLEWARE_PASSWORD=middleware
#signal
export MIDDLEWARE_TASK_ID=699bad54-e45e-4d1f-bb8e-293bc964f71d
#reference
#export ROS_DOMINA_ID=34
#export MIDDLEWARE_TASK_ID=6d7b20ad-8459-4f00-91f4-816dded6395c
export MIDDLEWARE_ROBOT_ID=300c719a-1c06-4500-a13a-c2e20592b273
# TOPICS TO SEND FROM ROBOT TO NETWORK APPLICATION
export TOPICS_FROM_SERVER='[{"name": "/robot/robotnik_base_control/cmd_vel_unstamped", "type": "geometry_msgs/msg/Twist"}]'
# TOPICS TO SEND FROM NETWORK APPLICATION BACK TO ROBOT
export TOPICS_TO_SERVER='[{"name": "/robot/robotnik_base_control/odom", "type": "nav_msgs/msg/Odometry"},{"name": "/image_raw", "type": "sensor_msgs/msg/Image"}]'
# RUN RELAY CLIENT
python3 -m era_5g_relay_network_application.client
