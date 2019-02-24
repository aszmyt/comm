#include "ros/ros.h"
#include "std_msgs/Float32.h"


const int LOOP_RATE = 10;


int main(int argc, char **argv)
{

  ros::init(argc, argv, "time_gen");
  ros::NodeHandle n;
  ros::Publisher time_out = n.advertise<std_msgs::Float32>("time", 1000);
  ros::Rate loop_rate(LOOP_RATE);
  double count = 0.0;
  while (ros::ok())
  {
    std_msgs::Float32 msg;

    msg.data = count;  
    //ROS_INFO("%s", msg.data.c_str());
    time_out.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    count += 1.0/LOOP_RATE;
  }

  return 0;
}
