#include "ros/ros.h"
#include "std_msgs/String.h"
#include "comm/control.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const comm::control::ConstPtr& msg)
{
  //std_msgs::String str = msg->name;
  ROS_INFO("I heard: [%f, %f]", msg->u1.data, msg->u2.data);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();

  return 0;
}
