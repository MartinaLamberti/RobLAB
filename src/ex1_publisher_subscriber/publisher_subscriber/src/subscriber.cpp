#include <ros/ros.h>
#include "publisher_subscriber_msgs/Message1.h"

void chatterCallback(const publisher_subscriber_msgs:: Message1& message)
{
  ROS_INFO("SUB = JOINT1: %f, JOINT2: %f, JOINT3: %f, JOINT4: %f, JOINT5: %f, JOINT6: %f\n", message.joint1, message.joint2, message.joint3, message.joint4, message.joint5, message.joint6);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle nodeHandle;

  ros::Subscriber subscriber = nodeHandle.subscribe("chatter", 10, chatterCallback);
  ros::spin();
  // ros::shutdown();
  return 0;
}