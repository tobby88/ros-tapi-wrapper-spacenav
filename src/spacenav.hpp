#ifndef SPACENAV_H
#define SPACENAV_H

#include "ros/ros.h"
#include "std_msgs/Header.h"

class Spacenav
{
public:
  // Constructor/Destructor
  Spacenav(ros::NodeHandle* nh);
  ~Spacenav();

  // Public member functions
  bool Connect();

private:
  // Private member variables
  std_msgs::Header header;
  ros::ServiceClient helloClient;
  ros::NodeHandle* nh;
};

#endif // SPACENAV_H
