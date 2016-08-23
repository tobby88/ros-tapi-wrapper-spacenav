#ifndef SPACENAV_H
#define SPACENAV_H

#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "tobbyapi_msgs/Feature.h"
#include <sensor_msgs/Joy.h>
#include <string>
#include <thread>
#include <vector>

class Spacenav
{
public:
  // Constructor/Destructor
  Spacenav(ros::NodeHandle* nh);
  ~Spacenav();

private:
  // Private member variables
  std::vector<tobbyapi_msgs::Feature> featureMsgs;
  std::vector<std::string> featureUUIDs;
  bool firstRun;
  std_msgs::Header header;
  unsigned long heartbeatInterval;
  std::thread* heartbeatThread;
  ros::ServiceClient helloClient;
  ros::NodeHandle* nh;
  std::string uuid;

  // Private member functions
  bool connect();
  void generateFeatureMsgs();
  std::string generateUUID();
  void heartbeat();
  void loadUUIDs();
};

#endif // SPACENAV_H
