#include "ros/ros.h"
#include "comm/changeInitialConditions.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "changeInitialConditionsClient");
  if (argc != 4)
  {
    ROS_INFO("usage: changeInitialConditionsClient X Y THETA");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<comm::changeInitialConditions>("changeInitialConditions");
  comm::changeInitialConditions srv;
  srv.request.x = atoll(argv[1]);
  srv.request.y = atoll(argv[2]);
  srv.request.theta = atoll(argv[3]);
  client.call(srv);
  /*if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
    }*/

  return 0;
}
