#ifndef SPACENAV_H
#define SPACENAV_H

#include "ros/ros.h"
#include "std_msgs/Header.h"
#include <string>
#include <thread>

class Spacenav
{
public:
  // Constructor/Destructor
  Spacenav(ros::NodeHandle* nh);
  ~Spacenav();

private:
  // Private member variables
  bool firstRun;
  std_msgs::Header header;
  unsigned long heartbeatInterval;
  std::thread* heartbeatThread;
  ros::ServiceClient helloClient;
  ros::NodeHandle* nh;
  std::string uuid;

  // Private member functions
  bool connect();
  void heartbeat();
  void loadUUID();
};

#endif // SPACENAV_H
