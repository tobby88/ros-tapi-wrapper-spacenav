/******************************************************************************
*  Copyright (C) 2016 by Tobias Holst                                         *
*                                                                             *
*  This file is part of tapi_wrapper_spacenav.                                *
*                                                                             *
*  tapi_wrapper_spacenav is free software: you can redistribute it and/or     *
*  modify it under the terms of the GNU General Public License as published   *
*  by the Free Software Foundation, either version 3 of the License, or       *
*  (at your option) any later version.                                        *
*                                                                             *
*  tapi_wrapper_spacenav is distributed in the hope that it will be useful,   *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of             *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
*  GNU General Public License for more details.                               *
*                                                                             *
*  You should have received a copy of the GNU General Public License along    *
*  with tapi_wrapper_spacenav.  If not, see <http://www.gnu.org/licenses/>.   *
*                                                                             *
*  Diese Datei ist Teil von tapi_wrapper_spacenav.                            *
*                                                                             *
*  tapi_wrapper_spacenav ist Freie Software: Sie können es unter den          *
*  Bedingungen der GNU General Public License, wie von der Free Software      *
*  Foundation Version 3 der Lizenz oder (nach Ihrer Wahl) jeder späteren      *
*  veröffentlichten Version, weiterverbreiten und/oder modifizieren.          *
*                                                                             *
*  tapi_wrapper_spacenav wird in der Hoffnung, dass es nützlich sein wird,    *
*  aber OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite    *
*  Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK  *
*  Siehe die GNU General Public License für weitere Details.                  *
*                                                                             *
*  Sie sollten eine Kopie der GNU General Public License zusammen mit diesem  *
*  Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>. *
*******************************************************************************/

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
