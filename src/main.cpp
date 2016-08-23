#include "ros/ros.h"
#include "spacenav.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TobbyAPI_Wrapper_Spacenav");
  ros::NodeHandle nh;
  Spacenav spacenav(&nh);
  while (ros::ok())
  {
    spacenav.Connect();
  }

  return 0;
}
