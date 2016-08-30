#include "spacenav.hpp"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "tapi_msgs/Feature.h"

using namespace ros;
using namespace std;

namespace Tapi
{
// Constructor/Destructor

Spacenav::Spacenav(NodeHandle* nh) : nh(nh)
{
  apiPub = new Tapi::Publisher(nh, "wrapper_spacenav");

  spacenavPub[0] = apiPub->AddFeature(tapi_msgs::Feature::Type_AnalogValue, 1, "Linear X");
  spacenavPub[1] = apiPub->AddFeature(tapi_msgs::Feature::Type_AnalogValue, 1, "Linear Y");
  spacenavPub[2] = apiPub->AddFeature(tapi_msgs::Feature::Type_AnalogValue, 1, "Linear Z");
  spacenavPub[3] = apiPub->AddFeature(tapi_msgs::Feature::Type_AnalogValue, 1, "Angular X");
  spacenavPub[4] = apiPub->AddFeature(tapi_msgs::Feature::Type_AnalogValue, 1, "Angular Y");
  spacenavPub[5] = apiPub->AddFeature(tapi_msgs::Feature::Type_AnalogValue, 1, "Angular Z");
  spacenavPub[6] = apiPub->AddFeature(tapi_msgs::Feature::Type_Switch, 1, "Button 1");
  spacenavPub[7] = apiPub->AddFeature(tapi_msgs::Feature::Type_Switch, 1, "Button 2");

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
