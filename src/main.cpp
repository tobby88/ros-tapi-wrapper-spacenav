#include "ros/ros.h"
#include "spacenav.hpp"
#include <chrono>
#include <thread>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TobbyAPI_Wrapper_Spacenav");
  ros::NodeHandle nh;
  Spacenav spacenav(&nh);
  while (ros::ok())
  {
    this_thread::sleep_for(chrono::milliseconds(100));
  }

  return 0;
}
