#!/bin/sh
gnome-terminal --window \
 --title=roscore --command "bash -c \"roscore; exec bash\"" \
--tab  --title=Gazebo --command "bash -c \"cd ~/catkin_ws;roslaunch leo_erc_gazebo leo_marsyard2021.launch --wait; exec bash\"" \
--tab  --title=Rviz --command "bash -c \"cd ~/catkin_ws;roslaunch leo_erc_viz rviz.launch --wait; exec bash\"" \

