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
  bool connected = false;
  while (!connected)
  {
    connected = spacenav.Connect();
    this_thread::sleep_for(chrono::milliseconds(1000));
  }
  while (ros::ok())
  {
  }

  return 0;
}
