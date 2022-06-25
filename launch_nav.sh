#!/bin/sh
gnome-terminal --window \
       --title=RtabMap --command "bash -c \"cd ~/catkin_ws;roslaunch leo_nav my_create_occupancy.launch --wait; exec bash\"" \
--tab  --title=ArTagDetection --command "bash -c \"cd ~/catkin_ws;roslaunch leo_nav pr2_indiv.launch --wait; exec bash\"" \
--tab  --title=CheckErrorInOdometry --command "bash -c \"cd ~/catkin_ws;rosrun tasks_utils check_odom_error.py; exec bash\"" \
--tab  --title=ArTagPose --command "bash -c \"cd ~/catkin_ws;rosrun leo_nav TagEstimatePose.py; exec bash\"" \
--tab  --title=Filtering --command "bash -c \"cd ~/catkin_ws;roslaunch leo_nav pose_ekf.launch; exec bash\"" \
