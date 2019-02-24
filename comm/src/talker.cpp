#include "ros/ros.h"
#include "std_msgs/String.h"
#include "comm/control.h"

//#include <sstream>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<comm::control>("chatter", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    comm::control msg;
    msg.u1.data = 1.2;
    msg.u2.data = 3.4;
    //std_msgs::String str;

    /*std::stringstream ss;
    ss << "hello world " << count;
    msg.name = ss.str();*/
    // msg.name = str;
    //ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}
