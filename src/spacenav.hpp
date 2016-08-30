#ifndef SPACENAV_H
#define SPACENAV_H

#include <sensor_msgs/Joy.h>
#include "ros/ros.h"
#include "tapi_clientlib/publisher.hpp"

namespace Tapi
{
class Spacenav
{
public:
  // Constructor/Destructor
  Spacenav(ros::NodeHandle* nh);
  ~Spacenav();

private:
  // Private member variables
  Tapi::Publisher* apiPub;
  ros::NodeHandle* nh;
  ros::Publisher* spacenavPub[8];
  ros::Subscriber spacenavSub;

  // Private member functions
  void forwardData(const sensor_msgs::Joy::ConstPtr& received);
};
}

#endif  // SPACENAV_H
