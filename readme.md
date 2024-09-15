# Utilities
Some scripts and files that could be useful during the development

## Scripts and nodes

### Firmware
 Scripts and nodes used to move the rover or rover components.
- **firmware_CC8**: CC8 firmware. Convert a twist message into velocities for the wheels and angles for the steering wheel. Some similar implementations [here](https://github.com/srmainwaring/curio/blob/baec5d6d82454d7f4859479c7d481657b1890d09/ackermann_drive_controller/src/ackermann_drive_controller.cpp#L647) and [here](https://github.com/Roger-random/Sawppy_Rover/blob/e53858b1f8efefa7f6a7ee1ef94031ad2bd2ba9d/esp32_sawppy/lib/rover/wheel_ackermann.c#L101-L141)
The code contains some parameters to enable or disbale some behaviors.
  - `wheel_radius=0.12` : Radius of the wheel [m].
  - `max_steer=math.pi/4`: Maximum steering angle [rad].
  - `MAX_V=1`: Maximum linear velocity [m/s].
  - `desired_in_place_velocity = math.pi/6`: Desired velocity for in-place rotation [rad/s].
  - `in_place_delay=2`: How much to wait for the wheel to be in position before starting in-place movement [s].
  - `min_wheel_velocity = 0.05`: Minimum velocity actuated for the driving wheels. If a lower speed is computed, it is clamped to 0. Used to avoid zzzzt sound.
  - `turning_ratio_threshold=0.6`: Threshold on v/w ratio to avoid turning radius too small in Ackermann steering.
  - `in_place_on_sharp_turn = True`: If true, the rover will rotate in place when turning with a small radius; otherwise, it will decrease the angular velocity.
  - `enable_steer_limit = False`: Enable steering angle limit. If true, the steering angle will be limited to `max_steer`.
  - `enable_lock_steer = True`: Enable lock steering. If true, the steering angle will be locked to the last angle when the rover is stopped.
  - `enable_steer_reset = True`: Enable steering reset. If true, the steering angle will be reset to 0 after some time when the rover is stopped.
  - `steer_reset_timeout= 3.0`: How many seconds to wait when the rover is stopped before resetting the wheels [s].
  - `enable_velocity_feedback = False`: Use a PD controller to adjust the velocity based on the feedback from an odometry message.
  - `velocity_feedback_KP = 0.5`: Proportional gain for the velocity feedback controller.
  - `velocity_feedback_KD = 0.01`: Derivative gain for the velocity feedback controller.

- **firmware_drilling**: A terminal interface used to operate the driling system and surface sampling system. 

- **navVelManager** a node that offer different services to swith from manual mode to autonomus mode for the navigation. To be used in conjunction with navigationTask node from nav_utils.
This node takes velocity commands from different topics and output on the main topic the one that corresponds to the current state of the rover.
It assumes that the firmware receives velocity commands on `/cmd_vel topic`, manual commands are sent on `/cmd_vel_manual` topic and autonomous navigation commands are sent on `/cmd_vel_auto` topic.
Calling the services `CC8_manual`, `CC8_auto` or `CC8_idle` will switch the state of the rover.
There is an additional state called by the service `CC8_lora` that uses the velocity received through Lora on the topic `/cmd_vel_lora`. 
See [com_utils](https://github.com/Sapienza-Technology/com_utils) package for details.
This node also change LED colors based on the state of the rover. This function is used when the firmware of the drilling system board is active.
### Sensors
Scripts and nodes used to start sensors and publish data to ROS.
- **camera_setup**: node used to start a usb camera and publish compressed iamge and camera info to ROS. to be launched from launch file (see my_camera_setup.launch)

### Other
Other nodes and scripts that are useful for the tasks.
- **capture_camera**; simply subscribe to an image topic and save images when a button is pressed. It was used to capture many images for calibration of the camera.
- energy_report_manager: read measurements from energy sensor and jetson library, plot them and send in topic. A draft implementation, to be used for pretty energy reports.
- check_odom_error: compute the error ( in 2D) between the real position of the robot (from Gazebo) and the estimated one.
- convert_gt: convert the global ground truth position from Gazebo to position with respect to starting position.
it receives from topic /ground_truth and republish the pose with respect to the starting position in the topic /ground_truth_local.
- plot_path : plot the path of the robot, getting data from multiple Odometry sources.
It accepts nav_msgs/Odometry topics. It uses diferent parameters (see related launch file for an example):
  - odom_topics: list of topics to subscribe to
  - colours: list of colors to use when plotting
  - saving_path: where to save the plot
- get_compressed_images: C++ node that subscribe to a pair of images from the zed camera and republish a compressed version, not useful, to remove.

### Old
Old scripts that were used on other rovers.
- firmware_velocity_LEO: firmware of our LEO rover (a.k.a. EVE)
- firmware_velocity: convert the linear and angular velocity to velocities of the wheels. Used for ERA.


#### Other scripts (not ROS nodes)
- **calibrate_camera.py**: from a  folder with images of a checkerboard (see pdf in the folder), calibrate the camera, output the camera parameters and save them to file in correct format. [remember to rename the file or it will be overwritten on the next calibration]. These files will be used by the launch files (see my_camera_setup.launch).
- find_number_to_press: small utility script used in 2023 maintenance task. write the numbers of the panel and it output a combination of numbers that give the desired sum.
- take_pictures: useful script that save images from a camera for calibration. this script does not interact with ROS, images from the camera are raw.


## Launch files
- **my_camera_setup**: setup a USB camera with ROS. remember to use the correct parameters, USB port and yaml file for the camera info.
- **realsense.launch**: launch a realsense camera with ROS. It is possible to select a configuration from a json file to increase or decrease accuracy or select the width and height. 
-  camera_setup: same as above but it uses an already existing library that warps the image. So a custom implementation was written. Not use unless for testing purposes
- localization_comparison.launch: get ground truth and different odometry sources,measure the error and plot the path
- Other launch files are not important, they are only tests

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
