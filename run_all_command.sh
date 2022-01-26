#!/bin/sh
cd ~
set -x -e
gnome-terminal -- sh -c 'roslaunch iq_sim lidar.launch' 1> /dev/null 2> /dev/null
gnome-terminal -- sh -c 'cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console' 1> /dev/null 2> /dev/null

sleep 30

gnome-terminal -- sh -c 'roslaunch iq_sim apm.launch' 1> /dev/null 2> /dev/null

gnome-terminal -- sh -c 'rosrun web_video_server web_video_server' 1> /dev/null 2> /dev/null

sleep 3

gnome-terminal -- sh -c 'rosrun image_view image_view image:=/webcam1/image_raw' 1> /dev/null 2> /dev/null
sleep 25

gnome-terminal -- sh -c 'rosrun iq_gnc waypoint.py' 1> /dev/null 2> /dev/null


gnome-terminal -- sh -c 'rosrun iq_gnc obs_avoid.py' 1> /dev/null 2> /dev/null