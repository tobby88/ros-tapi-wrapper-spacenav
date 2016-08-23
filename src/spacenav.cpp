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

bool Spacenav::Connect()
{
  tobbyapi_msgs::Hello hello;
  header.stamp = Time::now();
  header.seq++;
  hello.request.Header = header;
  hello.request.Name = "Spacenav";
  hello.request.UUID = uuid;
  hello.request.DeviceType = tobbyapi_msgs::HelloRequest::Type_SenderDevice;
  if (helloClient.call(hello))
  {
    if (hello.response.Status == tobbyapi_msgs::Hello::Response::StatusOK)
    {

      ROS_INFO("Connection established, Status OK, Heartbeat %u",
               hello.response.Heartbeat);
      return true;
    }
    else
    {
      ROS_INFO("Connection error, Heartbeat %u", hello.response.Heartbeat);
      return false;
    }
  }
  else
  {
    ROS_ERROR("Failed to establish connection to hello service");
    return false;
  }
  return false;
}

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
  this->uuid = uuid;
  return;
}
