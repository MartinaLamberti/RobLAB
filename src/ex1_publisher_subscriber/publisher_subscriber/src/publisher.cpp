#include <ros/ros.h>
#include "publisher_subscriber_msgs/Message1.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle nh;
  ros::Publisher chatterPublisher = nh.advertise<publisher_subscriber_msgs::Message1>("chatter", 1);
  ros::Rate loop_rate(10);

  while (ros::ok()){

    publisher_subscriber_msgs::Message1 message;
    message.joint1 = rand()%100; //genera un numero casuale tra 0 e 99
    message.joint2 = rand()%100;
    message.joint3 = rand()%100;
    message.joint4 = rand()%100;
    message.joint5 = rand()%100;
    message.joint6 = rand()%100;
    ROS_INFO("PUB = JOINT1: %f, JOINT2: %f, JOINT3: %f, JOINT4: %f, JOINT5: %f, JOINT6: %f\n", message.joint1, message.joint2, message.joint3, message.joint4, message.joint5, message.joint6);
    chatterPublisher.publish(message);
    ros::spinOnce();
    loop_rate.sleep();
  }
//ros::shutdown();
  return 0;
}