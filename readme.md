## Utilities
Some scripts and files taht could be useful during the development

### Scripts 
- check_odom_error: compute the error between the real position of the robot (from Gazebo) and the estimated one
- convert_gt: convert the global ground truth position from Gazebo to position with respect to starting position
- plot_path : plot the path of the robot, getting data from multiple Odometry sources
- firmware_velocity: convert the linear and angular velocity to velocities of the wheels
  
### Launch files
- localization_comparison.launch: get ground truth and different odometry sources,measure the error and plot the path

### file.sh
 - launch_env.sh starts: 
   - gazebo
   - rviz
 - launch_nav.sh starts:
   - Rtabmap
   - ArTag detection
   - ArTag pose estimation
   - Pose filtering
   - Error between real and estimated pose

### Other
- configurations: some config for Rviz