#include "ros/ros.h"
#include "comm/control.h"


const int LOOP_RATE = 1;


int main(int argc, char **argv)
{

  ros::init(argc, argv, "control_gen");
  ros::NodeHandle n;
  ros::Publisher ctrl_out = n.advertise<comm::control>("control", 1000);
  ros::Rate loop_rate(LOOP_RATE);
  bool positiveU2 = true;
  while (ros::ok())
  {
    comm::control msg;
    msg.u1.data = 0.2;  
    if(positiveU2)
      msg.u2.data = 0.1;
    else
      msg.u2.data = -0.1;
    positiveU2 = !positiveU2;
    ctrl_out.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
