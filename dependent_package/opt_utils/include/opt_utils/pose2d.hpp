/* Copyright (c) 2017, Yu Zhang, Intelligent Vehicle Research Center(IVRC),
 * Beijing Institute of Technology.
 * All rights reserved.
 *        Files: pose2d.hpp
 *   Created on: 09, 26, 2017
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */

#ifndef OPT_UTILS_POSE_2D_HPP
#define OPT_UTILS_POSE_2D_HPP

#include <list>
#include <vector>

#include "vector2d.hpp"

namespace hmpl {

class Pose2D {
 public:
    // the position
    Vector2D<double> position;

    // the current orientation
    double orientation;

    // most basic constructor
    Pose2D();

    // copy constructor
    Pose2D(const Pose2D&);

    // explicit constructor
    Pose2D(const Vector2D<double>&, double);

    // explicit constructor
    Pose2D(double, double, double);

    // assignment operator overloading
    void operator=(const Pose2D&);

    // equals to operator overloading
    bool operator==(const Pose2D&);

    // different operator overloading
    bool operator!=(const Pose2D&);
};

class PoseArray {
 public:
    // the pose array
    std::vector<Pose2D> poses;
};

typedef PoseArray* PoseArrayPtr;

class PoseList {
 public:
    // the pose list
    std::list<Pose2D> poses;
};

typedef PoseList* PoseListPtr;
}
#endif  // OPT_UTILS_POSE_2D_HPP
