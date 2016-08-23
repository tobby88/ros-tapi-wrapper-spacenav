#include "ros/ros.h"
#include "spacenav.hpp"

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TobbyAPI_Wrapper_Spacenav");
  ros::NodeHandle nh;
  Spacenav* spacenav = new Spacenav(&nh);
  while (ros::ok())
  {
    ros::spin();
  }
  delete spacenav;
  return 0;
}
