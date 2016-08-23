#include "spacenav.hpp"
#include "tobbyapi_msgs/Hello.h"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <uuid/uuid.h>

using namespace ros;
using namespace std;

// Constructor/Destructor

Spacenav::Spacenav(NodeHandle* nh)
{
  this->nh = nh;
  firstRun = true;
  loadUUIDs();
  helloClient = nh->serviceClient<tobbyapi_msgs::Hello>("TobbyAPI/HelloServ");
  heartbeatThread = new thread(&Spacenav::heartbeat, this);
}

Spacenav::~Spacenav()
{
  heartbeatThread->join();
  delete heartbeatThread;
}

// Private member functions

bool Spacenav::connect()
{
  bool status = false;
  tobbyapi_msgs::Hello hello;
  header.stamp = Time::now();
  header.seq++;
  hello.request.Header = header;
  hello.request.Name = "Spacenav";
  hello.request.UUID = uuid;
  hello.request.DeviceType = tobbyapi_msgs::HelloRequest::Type_SenderDevice;
  if (helloClient.call(hello))
  {
    status = hello.response.Status;
    if (firstRun)
    {
      if (status)
        ROS_INFO("Connection established, Status OK, Heartbeat %u",
                 hello.response.Heartbeat);
      else
        ROS_INFO("Connection error, Heartbeat %u", hello.response.Heartbeat);
      firstRun = false;
    }
    heartbeatInterval = hello.response.Heartbeat;
  }
  else
  {
    ROS_ERROR("Failed to establish connection to hello service");
    status = false;
  }

  return status;
}

void Spacenav::heartbeat()
{
  while (ros::ok())
  {
    bool success = false;
    unsigned long waitmsOnError = 1000;
    success = connect();
    if (!success)
    {
      this_thread::sleep_for(chrono::milliseconds(waitmsOnError));
      continue;
    }
    else
      this_thread::sleep_for(chrono::milliseconds(heartbeatInterval));
  }
}

void Spacenav::loadUUIDs()
{
  // Read (or generate, if not existing) Device-UUID
  string uuid;
  string homedir = getenv("HOME");
  string filename = homedir + "/.ros/tobbyapi_wrapper_spacenav_dev_uuid.txt";
  ifstream uuidFileInput;
  uuidFileInput.open(filename);
  if (uuidFileInput.is_open())
  {
    getline(uuidFileInput, uuid);
  }
  uuidFileInput.close();
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

  // Now also look for Feature-UUIDs
  filename = homedir + "/.ros/tobbyapi_wrapper_spacenav_feature_uuids.txt";
  uuidFileInput.open(filename);
  if (uuidFileInput.is_open())
  {
    featureUUIDs.clear();
    while (getline(uuidFileInput, uuid))
      featureUUIDs.push_back(uuid);
  }
  uuidFileInput.close();
  if (featureUUIDs.size() != 8)
  {
    ofstream uuidFileOutput;
    uuidFileOutput.open(filename, ios::app);
    while (featureUUIDs.size() < 8)
    {
      uuid_t uuidt;
      uuid_generate_random(uuidt);
      char uuid_array[37];
      uuid_unparse(uuidt, uuid_array);
      uuid = uuid_array;
      replace(uuid.begin(), uuid.end(), '-', '_');
      featureUUIDs.push_back(uuid);
      uuidFileOutput << uuid << "\n";
    }
    uuidFileOutput.close();
  }
  return;
}
