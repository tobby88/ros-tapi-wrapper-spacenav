#include "spacenav.hpp"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

Spacenav::Spacenav(ros::NodeHandle* nh) : nh(nh)
{
  apiPub = new Tapi::Publisher(nh, "Spacenav Wrapper");

  spacenavPub[0] = apiPub->AddFeature<std_msgs::Float64>("Linear X", 1);
  spacenavPub[1] = apiPub->AddFeature<std_msgs::Float64>("Linear Y", 1);
  spacenavPub[2] = apiPub->AddFeature<std_msgs::Float64>("Linear Z", 1);
  spacenavPub[3] = apiPub->AddFeature<std_msgs::Float64>("Angular X", 1);
  spacenavPub[4] = apiPub->AddFeature<std_msgs::Float64>("Angular Y", 1);
  spacenavPub[5] = apiPub->AddFeature<std_msgs::Float64>("Angular Z", 1);
  spacenavPub[6] = apiPub->AddFeature<std_msgs::Bool>("Button 1", 1);
  spacenavPub[7] = apiPub->AddFeature<std_msgs::Bool>("Button 2", 1);
  spacenavPub[8] = apiPub->AddFeature<sensor_msgs::Joy>("[Optional] Full Joy Message", 1);

  spacenavSub = nh->subscribe("spacenav/joy", 1, &Spacenav::forwardData, this);
}

Spacenav::~Spacenav()
{
  spacenavSub.shutdown();
  delete apiPub;
}

// Private member functions

void Spacenav::forwardData(const sensor_msgs::Joy::ConstPtr& received)
{
  spacenavPub[8]->publish(received);
  for (int i = 0; i < 6; i++)
  {
    std_msgs::Float64 forward;
    forward.data = received->axes[i];
    spacenavPub[i]->publish(forward);
  }
  std_msgs::Bool forward;
  forward.data = received->buttons[0];
  spacenavPub[6]->publish(forward);
  forward.data = received->buttons[1];
  spacenavPub[7]->publish(forward);
}
}
