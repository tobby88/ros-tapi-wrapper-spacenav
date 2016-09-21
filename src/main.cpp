/******************************************************************************
 *  Copyright (C) 2016 by Tobias Holst                                        *
 *                                                                            *
 *  This file is part of tapi_wrapper_spacenav.                               *
 *                                                                            *
 *  tapi_wrapper_spacenav is free software: you can redistribute it and/or    *
 *  modify it under the terms of the GNU General Public License as published  *
 *  by the Free Software Foundation, either version 3 of the License, or      *
 *  (at your option) any later version.                                       *
 *                                                                            *
 *  tapi_wrapper_spacenav is distributed in the hope that it will be useful,  *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of            *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
 *  GNU General Public License for more details.                              *
 *                                                                            *
 *  You should have received a copy of the GNU General Public License along   *
 *  with tapi_wrapper_spacenav.  If not, see <http://www.gnu.org/licenses/>.  *
 *                                                                            *
 *  Diese Datei ist Teil von tapi_wrapper_spacenav.                           *
 *                                                                            *
 *  tapi_wrapper_spacenav ist Freie Software: Sie können es unter den         *
 *  Bedingungen der GNU General Public License, wie von der Free Software     *
 *  Foundation Version 3 der Lizenz oder (nach Ihrer Wahl) jeder späteren     *
 *  veröffentlichten Version, weiterverbreiten und/oder modifizieren.         *
 *                                                                            *
 *  tapi_wrapper_spacenav wird in der Hoffnung, dass es nützlich sein wird,   *
 *  aber OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite   *
 *  Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK *
 *  Siehe die GNU General Public License für weitere Details.                 *
 *                                                                            *
 *  Sie sollten eine Kopie der GNU General Public License zusammen mit diesem *
 *  Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.*
 ******************************************************************************/

/*!
 * \defgroup tapi_wrapper_spacenav tapi_wrapper_spacenav
 * \file main.cpp
 * \ingroup tapi_wrapper_spacenav
 * \author Tobias Holst
 * \date 23 Aug 2016
 * \brief Main function of tapi_wrapper_spacenav
 */

#include "ros/ros.h"
#include "spacenav.hpp"

/*!
 * \brief Main function of tapi_wrapper_spacenav
 *
 * Main function of tapi_wrapper_spacenav to initialize ROS, its NodeHandle and then create the Tapi::Spacenav object to
 * start the wrapper.
 * \param argc Number of arguments when started from the console
 * \param argv \c char pointer to the \c char arrays of the given arguments
 * \return 0 when exited correctly
 * \see Tapi::Spacenav
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "TobbyAPI_Wrapper_Spacenav");
  ros::NodeHandle nh;
  Tapi::Spacenav* spacenav = new Tapi::Spacenav(&nh);
  while (ros::ok())
    ros::spinOnce();
  delete spacenav;
  return 0;
}
