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
 * \file spacenav.hpp
 * \ingroup tapi_wrapper_spacenav
 * \author Tobias Holst
 * \date 23 Aug 2016
 * \brief Declaration of the Tapi::Spacenav-class and definition of its member variables
 */

#ifndef SPACENAV_H
#define SPACENAV_H

#include <sensor_msgs/Joy.h>
#include "ros/ros.h"
#include "tapi_lib/publisher.hpp"

namespace Tapi
{
class Spacenav
{
  /*!
   * \brief Wrapper to connect the spacenav via tapi_core
   *
   * This class wraps Joy-messages from the Spacenav_node to tapi-compliant publishers. Users have the choice to connect
   * the full Joy-messsage or all axes seperately.
   * \author Tobias Holst
   * \version 1.2.1
   */
public:
  // Constructor/Destructor

  /*!
   * \brief Create a Tapi::Spacenav object to wrap the data from the spacenav_node. There should be only one of this
   * type!
   * \param nh Pointer to a \c ros::NodeHandle created outside of this class
   */
  Spacenav(ros::NodeHandle* nh);

  //! Shutdown publishers and free memory
  ~Spacenav();

private:
  // Private member variables

  //! \c tapi_lib based Publisher object to create Tapi compliant Publishers
  Tapi::Publisher* apiPub;

  //! NodeHandle-pointer necessary to create subscribers, publishers and services.
  ros::NodeHandle* nh;

  /*!
   * \brief Pointer to publishers to publish the current data of the spacenav
   * \see Tapi::Spacenav::forwardData
   */
  ros::Publisher* spacenavPub[9];

  /*!
   * \brief Subscriber to listen on the spacenav_node
   * \see Tapi::Spacenav::forwardData
   */
  ros::Subscriber spacenavSub;

  // Private member functions

  /*!
   * \brief Forward the data from the spacenav_node as Tapi-compliant publishers
   *
   * This function receives messages from the spacenav_node and then re-publishes them. On one topic the whole
   * Joy-message is redstributed, on the other eight publishers the single axes of the spacenav (six axes) and the two
   * button topics are published.
   * \param received The message waiting in the ros message queue where the values of the spacenav are stored
   */
  void forwardData(const sensor_msgs::Joy::ConstPtr& received);
};
}

#endif  // SPACENAV_H
