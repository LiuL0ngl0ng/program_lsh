/**
 *
 *  \file
 *  \brief Wrapping swiftnav with ROS message types to support LLH->ENU
 *         and LLH<->ECEF conversions
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND
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

#include "gps_utils/enu.h"

#include <boost/bind.hpp>
#include <string>

extern "C" {
#include "gps_utils/coord_system.h"
}

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#define TO_RADIANS (M_PI / 180)
#define TO_DEGREES (180 / M_PI)

extern "C" int xerbla_(char* srname, int* info) {
    ROS_ERROR("On entry to %6s, parameter number %2i had an illegal value",
              srname, *info);
}

namespace enu {

static void fix_to_ecef(const sensor_msgs::NavSatFix& fix, double ecef[3]) {
    double llh[3] = {fix.latitude * TO_RADIANS, fix.longitude * TO_RADIANS,
                     fix.altitude};
    wgsllh2ecef(llh, ecef);
}

void fix_to_point(const sensor_msgs::NavSatFix& fix,
                  const sensor_msgs::NavSatFix& datum,
                  geometry_msgs::Point* point_ptr) {
    // Convert reference LLH-formatted datum to ECEF format
    double ecef_datum[3];
    fix_to_ecef(datum, ecef_datum);
    // Prepare the appropriate input vector to convert the input latlon
    // to an ECEF triplet.
    double llh[3] = {fix.latitude * TO_RADIANS, fix.longitude * TO_RADIANS,
                     fix.altitude};
    double ecef[3];
    wgsllh2ecef(llh, ecef);
    // ECEF triplet is converted to north-east-down (NED), by combining it
    // with the ECEF-formatted datum point.
    double ned[3];
    wgsecef2ned_d(ecef, ecef_datum, ned);
    // Output data
    point_ptr->x = ned[1];
    point_ptr->y = ned[0];
    point_ptr->z = -ned[2];
}

void point_to_fix(const geometry_msgs::Point& point,
                  const sensor_msgs::NavSatFix& datum,
                  sensor_msgs::NavSatFix* fix_ptr) {
    // Convert reference LLH-formatted datum to ECEF format
    double ecef_datum[3];
    fix_to_ecef(datum, ecef_datum);

    // Prepare NED vector from ENU coordinates, perform conversion in
    // libswiftnav
    // library calls.
    double ned[3] = {point.y, point.x, -point.z};

    double ecef[3];
    wgsned2ecef_d(ned, ecef_datum, ecef);

    double llh_raw[3];
    wgsecef2llh(ecef, llh_raw);

    // Output Fix message. Convert radian latlon output back to degrees.
    fix_ptr->latitude = llh_raw[0] * TO_DEGREES;
    fix_ptr->longitude = llh_raw[1] * TO_DEGREES;
    fix_ptr->altitude = llh_raw[2];
}

void enu2ecef_matrix(const double datum_llh[3], double enu_datum_ecef[3],
                     double rotation_ECEF_ENU[3][3]) {
    wgsllh2ecef(datum_llh, enu_datum_ecef);

    /* Reminder that the earth-centre coordinate frame is a right-handed
    * Cartesian
    * frame with an origin located at the center of the earth. Z axis goes
    * through
    * True North and the X-axis passes through the prime meridian.
    * The sines and cosines are calculated using Pythagorean theorem.
    * https://en.wikipedia.org/wiki/ECEF */
    double hyp_lon, hyp_lat;
    double sin_lat, cos_lat, sin_lon, cos_lon;
    hyp_lon = sqrt(enu_datum_ecef[0] * enu_datum_ecef[0] +
                   enu_datum_ecef[1] * enu_datum_ecef[1]);
    hyp_lat = sqrt(hyp_lon * hyp_lon + enu_datum_ecef[2] * enu_datum_ecef[2]);
    sin_lat = enu_datum_ecef[2] / hyp_lat;
    cos_lat = hyp_lon / hyp_lat;
    sin_lon = enu_datum_ecef[1] / hyp_lon;
    cos_lon = enu_datum_ecef[0] / hyp_lon;

    /*
     * Refer to
     * http://www.navipedia.net/index.php/Transformations_between_ECEF_and_ENU_coordinates
     * specifically to equation number 3, noting phi is latitude and lamda is
     * longitude
     */
    rotation_ECEF_ENU[0][0] = -sin_lon;
    rotation_ECEF_ENU[0][1] = -sin_lat * cos_lon;
    rotation_ECEF_ENU[0][2] = cos_lat * cos_lon;
    rotation_ECEF_ENU[1][0] = cos_lon;
    rotation_ECEF_ENU[1][1] = -sin_lat * sin_lon;
    rotation_ECEF_ENU[1][2] = cos_lat * sin_lon;
    rotation_ECEF_ENU[2][0] = 0;
    rotation_ECEF_ENU[2][1] = cos_lat;
    rotation_ECEF_ENU[2][2] = sin_lat;
}

void ecef2enu_matrix(const double datum_llh[3], double ecef_origin_enu[3],
                     double rotation_ENU_ECEF[3][3]) {
    // Get enu2ecef transform at datum
    double enu_datum_ecef[3] = {0.0, 0.0, 0.0};
    double rotation_ECEF_ENU[3][3] = {
            {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    ;
    enu2ecef_matrix(datum_llh, enu_datum_ecef, rotation_ECEF_ENU);

    // Construct transform as inverse of enu2ecef transform
    // Affine inverse of [R | t]^(-1) = [ R^T | - R^T * t]

    // TODO(msmart/benskikos) - Move these functions to somewhere where we have
    // matrix classes and ownership since it is ours. Manual transposition is
    // undesirable. The below should be 2 lines:
    // R_new = R.transpose(); t_new = - R.transpose()*t;
    rotation_ENU_ECEF[0][0] = rotation_ECEF_ENU[0][0];
    rotation_ENU_ECEF[0][1] = rotation_ECEF_ENU[1][0];
    rotation_ENU_ECEF[0][2] = rotation_ECEF_ENU[2][0];
    rotation_ENU_ECEF[1][0] = rotation_ECEF_ENU[0][1];
    rotation_ENU_ECEF[1][1] = rotation_ECEF_ENU[1][1];
    rotation_ENU_ECEF[1][2] = rotation_ECEF_ENU[2][1];
    rotation_ENU_ECEF[2][0] = rotation_ECEF_ENU[0][2];
    rotation_ENU_ECEF[2][1] = rotation_ECEF_ENU[1][2];
    rotation_ENU_ECEF[2][2] = rotation_ECEF_ENU[2][2];

    // Affine inverse translation component: -R_inverse * b
    ecef_origin_enu[0] = -rotation_ENU_ECEF[0][0] * enu_datum_ecef[0] -
                         rotation_ENU_ECEF[0][1] * enu_datum_ecef[1] -
                         rotation_ENU_ECEF[0][2] * enu_datum_ecef[2];

    ecef_origin_enu[1] = -rotation_ENU_ECEF[1][0] * enu_datum_ecef[0] -
                         rotation_ENU_ECEF[1][1] * enu_datum_ecef[1] -
                         rotation_ENU_ECEF[1][2] * enu_datum_ecef[2];

    ecef_origin_enu[2] = -rotation_ENU_ECEF[2][0] * enu_datum_ecef[0] -
                         rotation_ENU_ECEF[2][1] * enu_datum_ecef[1] -
                         rotation_ENU_ECEF[2][2] * enu_datum_ecef[2];
}

}  // namespace enu
