/* Copyright (c) 2017, Yu Zhang, Intelligent Vehicle Research Center(IVRC),
 * Beijing Institute of Technology.
 * All rights reserved.
 *        Files: motion.hpp
 *   Created on: 09, 26, 2017
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#ifndef OPT_UTILS_MOTION_HPP
#define OPT_UTILS_MOTION_HPP

namespace hmpl {
// define the steering values
enum Steer { RSTurnLeft, RSStraight, RSTurnRight };

const static unsigned int NumSteering = 3;

// define the Gears
enum Gear { ForwardGear, BackwardGear };

// define the sizes
const static unsigned int NumGears = 2;
}

#endif  // OPT_UTILS_MOTION_HPP
