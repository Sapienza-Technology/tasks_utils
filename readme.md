## Utilities
Some scripts and files that could be useful during the development

### Scripts 
- **firmware_CC8**: CC8 firmware. Convert a twist message into velocities for the wheels and angles for the steering wheel. Some similar implementations [here](https://github.com/srmainwaring/curio/blob/baec5d6d82454d7f4859479c7d481657b1890d09/ackermann_drive_controller/src/ackermann_drive_controller.cpp#L647) and [here](https://github.com/Roger-random/Sawppy_Rover/blob/e53858b1f8efefa7f6a7ee1ef94031ad2bd2ba9d/esp32_sawppy/lib/rover/wheel_ackermann.c#L101-L141)
- firmware_velocity_LEO: firmware of our LEO rover (a.k.a. EVE)
- firmware_velocity: convert the linear and angular velocity to velocities of the wheels. Used for ERA.
- **camera_setup**: node used to start a usb camera and publish compressed iamge and camera info to ROS. to be launched from launch file (see my_camera_setup.py)
- **capture_camera**; simply subscribe to an image topic and save images when a button is pressed. Used to capture many images for calibration
- energy_report_manager: read measurements from energy sensor and jetson library, plot them and send in topic. A draft implementation, to be used for pretty energy reports.
- firmware_drilling: a draft implementation to be used to move and operate the drill, it should work but it was never tested
- check_odom_error: compute the error between the real position of the robot (from Gazebo) and the estimated one
- convert_gt: convert the global ground truth position from Gazebo to position with respect to starting position
- plot_path : plot the path of the robot, getting data from multiple Odometry sources
- get_compressed_images: C++ node that subscribe to a pair of images from the zed camera and republish a compressed version, not useful, to remove.
- 
#### not ros
- **calibrate_camera.py**: froma  folder with images of a checkerboard (see pdf in the folder), calibrate the camera, output the camera parameters and save them to file in correct format. they will be used by the launch files.
- find_number_to_press: used for the maintenance task. write the numbers of the panel and it output a combination of numbers that give the desired sum.
- take_pictures: useful script that save images from a camera for calibration. this script does not interact with ROS, images from the camera are raw.
### Launch files
- **my_camera_setup**: setup a USB camera with ROS. remember to use the correct parameters, USB port and yaml file for the camera info.
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
