#include "spacenav.hpp"
#include <uuid/uuid.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "tobbyapi_msgs/Hello.h"

using namespace ros;
using namespace std;

// Constructor/Destructor

Spacenav::Spacenav(NodeHandle* nh)
{
  this->nh = nh;
  firstRun = true;
  loadUUIDs();
  generateFeatureMsgs();

  helloClient = nh->serviceClient<tobbyapi_msgs::Hello>("TobbyAPI/HelloServ");
  heartbeatThread = new thread(&Spacenav::heartbeat, this);

  for (int i = 0; i < 6; i++)
    spacenavPub[i] = nh->advertise<std_msgs::Float64>("TobbyAPI/" + uuid + "/" + featureUUIDs[i], 1);
  spacenavPub[6] = nh->advertise<std_msgs::Bool>("TobbyAPI/" + uuid + "/" + featureUUIDs[6], 1);
  spacenavPub[7] = nh->advertise<std_msgs::Bool>("TobbyAPI/" + uuid + "/" + featureUUIDs[7], 1);

  spacenavSub = nh->subscribe("spacenav/joy", 1, &Spacenav::forwardData, this);
}

Spacenav::~Spacenav()
{
  heartbeatThread->join();
  delete heartbeatThread;
  spacenavSub.shutdown();
  for (int i = 0; i < 8; i++)
    spacenavPub[i].shutdown();
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
  hello.request.Features = featureMsgs;
  if (helloClient.call(hello))
  {
    status = hello.response.Status;
    if (firstRun)
    {
      if (status)
        ROS_INFO("Connection established, Status OK, Heartbeat %u", hello.response.Heartbeat);
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

void Spacenav::forwardData(const sensor_msgs::Joy::ConstPtr& received)
{
  for (int i = 0; i < 6; i++)
  {
    std_msgs::Float64 forward;
    forward.data = received->axes[i];
    spacenavPub[i].publish(forward);
  }
  std_msgs::Bool forward;
  forward.data = received->buttons[0];
  spacenavPub[6].publish(forward);
  forward.data = received->buttons[1];
  spacenavPub[7].publish(forward);
}

void Spacenav::generateFeatureMsgs()
{
  tobbyapi_msgs::Feature featureMsgs[8];
  for (int i = 0; i < 8; i++)
  {
    featureMsgs[i].UUID = featureUUIDs[i];
    featureMsgs[i].FeatureType = tobbyapi_msgs::Feature::Type_AnalogValue;
  }
  featureMsgs[0].Name = "Linear X";
  featureMsgs[1].Name = "Linear Y";
  featureMsgs[2].Name = "Linear Z";
  featureMsgs[3].Name = "Angular X";
  featureMsgs[4].Name = "Angular Y";
  featureMsgs[5].Name = "Angular Z";
  featureMsgs[6].FeatureType = tobbyapi_msgs::Feature::Type_Switch;
  featureMsgs[6].Name = "Button 1";
  featureMsgs[7].FeatureType = tobbyapi_msgs::Feature::Type_Switch;
  featureMsgs[7].Name = "Button 2";
  for (int i = 0; i < 8; i++)
    this->featureMsgs.push_back(featureMsgs[i]);
}

string Spacenav::generateUUID()
{
  uuid_t uuidt;
  char uuid_array[37];
  string uuid_string;
  uuid_generate_random(uuidt);
  uuid_unparse(uuidt, uuid_array);
  uuid_string = uuid_array;
  replace(uuid_string.begin(), uuid_string.end(), '-', '_');
  return uuid_string;
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
    uuid = generateUUID();
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
      uuid = generateUUID();
      featureUUIDs.push_back(uuid);
      uuidFileOutput << uuid << "\n";
    }
    uuidFileOutput.close();
  }
  return;
}
