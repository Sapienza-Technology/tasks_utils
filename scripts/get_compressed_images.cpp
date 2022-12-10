#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

// Declare the image callback function
void imageCallback(const sensor_msgs::ImageConstPtr& msg, image_transport::Publisher& pub)
{
  // Publish the readable image on the output topic
  pub.publish(msg);
}
void imageCallback2(const sensor_msgs::ImageConstPtr& msg, image_transport::Publisher& pub2)
{
  // Publish the readable image on the output topic
  pub2.publish(msg);
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "republish_images");
  ros::NodeHandle nh;

  // Create an image_transport instance
  image_transport::ImageTransport it(nh);

  // Publish the readable images
  image_transport::Publisher pub = it.advertise("/zed2/left/image_rect_color_new", 1);
  image_transport::Publisher pub2 = it.advertise("/zed2/right/image_rect_color_new", 1);
  // Subscribe to the compressed images
  image_transport::Subscriber sub = it.subscribe("/zed2/left/image_rect_color", 1, boost::bind(imageCallback, _1, pub));
  image_transport::Subscriber sub2 = it.subscribe("/zed2/right/image_rect_color", 1, boost::bind(imageCallback2, _1, pub2));
  // Spin until the node is shut down
  ros::spin();

  return 0;
}