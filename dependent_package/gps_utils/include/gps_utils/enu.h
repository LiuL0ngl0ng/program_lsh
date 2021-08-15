/**
 *
 *  \file
 *  \brief Functions for conversion between LLH and ENU coordinates
 *         (given a defined datum)
 *  \author Ryan Gariepy <rgariepy@clearpathrobotics.com>
 *
 *  \copyright Copyright (c) 2013, Clearpath Robotics, Inc.
 *
 *  All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#ifndef INCLUDE_ENU_ENU_H_
#define INCLUDE_ENU_ENU_H_

#include "ros/message_forward.h"

namespace sensor_msgs { ROS_DECLARE_MESSAGE(NavSatFix); }

namespace geometry_msgs { ROS_DECLARE_MESSAGE(Point); }

namespace enu {

/**
 * \brief Converts an LLH coordinate into the corresponding ENU coordinate
 *
 * Requires a datum (passed as a NavSatFix)
 * \param[in] fix NavSatFix with a valid latitude, longitude, and altitude
 * representing the current position
 * \param[in] datum Datum point in LLH format
 * \param[out] point_ptr ENU of current position relative to datum
 */
void fix_to_point(const sensor_msgs::NavSatFix& fix,
                  const sensor_msgs::NavSatFix& datum,
                  geometry_msgs::Point* point_ptr);

/**
 * \brief Converts an ENU coordinate into the corresponding LLH coordinate
 *        Note that no population of the NavSatFix header occurs; that is the
 *        responsibility of the calling function.
 *
 * Requires a datum (passed as a NavSatFix)
 * \param [in] point ENU position with respect to datum point
 * \param [in] datum Datum point in LLH format
 * \param [out] fix_ptr LLH corresponding to ENU + datum combination
 */
void point_to_fix(const geometry_msgs::Point& point,
                  const sensor_msgs::NavSatFix& datum,
                  sensor_msgs::NavSatFix* fix_ptr);

/*!
 * @brief Computes the translation and rotation components of a transfrom from
 * ENU to ECEF for the provided ENU datum.
 *
 * @param[in] datum_llh Latitude Longitude Height (Altitude) coordinates of ENU
 *   datum, passed as [lat, long, height], with lat and long in radians, and
 *   height in meters.
 *
 * @param[out] enu_datum_ecef ENU datum coordinates [X Y Z] in ECEF frame in
 *   meters.
 *
 * @param[out] rotation_ECEF_ENU Rotation matrix transforming from the ENU
 *   frame to the ECEF frame.
 */
void enu2ecef_matrix(const double datum_llh[3], double enu_datum_ecef[3],
                     double rotation_ECEF_ENU[3][3]);

/*!
 * @brief Computes the translation and rotation components of a transfrom from
 * ECEF to ENU for the provided ENU datum.
 *
 * @param[in] datum_llh Latitude Longitude Height (Altitude) coordinates of ENU
 *   datum, passed as [lat, long, height], with lat and long in radians, and
 *   height in meters.
 *
 * @param[out] ecef_origin_enu Coordinates [E N U] of ECEF origin in ENU frame
 *
 * @param[out] rotation_ENU_ECEF Rotation matrix transforming from the ECEF
 *   frame to the ENU frame.
 */
void ecef2enu_matrix(const double datum_llh[3], double ecef_origin_enu[3],
                     double rotation_ENU_ECEF[3][3]);

}  // enu

#endif  // INCLUDE_ENU_ENU_H_
