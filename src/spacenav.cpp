#include "spacenav.hpp"
#include "tobbyapi_msgs/Hello.h"
#include <algorithm>
#include <fstream>
#include <uuid/uuid.h>

using namespace ros;
using namespace std;

// Constructor/Destructor

Spacenav::Spacenav(NodeHandle* nh)
{
  this->nh = nh;
  loadUUID();
  helloClient = nh->serviceClient<tobbyapi_msgs::Hello>("TobbyAPI/HelloServ");
}

Spacenav::~Spacenav() {}

// Public member functions

bool Spacenav::Connect() { return true; }

// Private member functions

void Spacenav::loadUUID()
{
  string uuid;
  string homedir = getenv("HOME");
  string filename = homedir + "/.ros/tobbyapi_wrapper_spacenav_dev_uuid.txt";
  ifstream uuidFileInput;
  uuidFileInput.open(filename);
  if (uuidFileInput.is_open())
  {
    getline(uuidFileInput, uuid);
    uuidFileInput.close();
  }
  if (uuid.empty())
  {
    ofstream uuidFileOutput;
    uuidFileOutput.open(filename);
    uuid_t uuidt;
    uuid_generate_random(uuidt);
    char uuid_array[37];
    uuid_unparse(uuidt, uuid_array);
    uuid = uuid_array;
    replace(uuid.begin(), uuid.end(), '-', '_');
    uuidFileOutput << uuid;
    uuidFileOutput.close();
  }
  ROS_INFO("%s", uuid.c_str());
  return;
}
