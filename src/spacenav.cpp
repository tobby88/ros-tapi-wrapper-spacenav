#include "spacenav.hpp"
#include "tobbyapi_msgs/Hello.h"

using namespace ros;

// Constructor/Destructor

Spacenav::Spacenav(NodeHandle* nh)
{
  this->nh = nh;
  helloClient = nh->serviceClient<tobbyapi_msgs::Hello>("TobbyAPI/HelloServ");
}

Spacenav::~Spacenav() {}

// Public member functions

bool Spacenav::Connect() { return true; }
