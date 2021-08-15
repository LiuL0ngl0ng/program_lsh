/* Copyright (c) 2017, Yu Zhang, Intelligent Vehicle Research Center(IVRC),
 * Beijing Institute of Technology.
 * All rights reserved.
 *        Files: state2d.hpp
 *   Created on: 09, 26, 2017
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#ifndef OPT_UTILS_STATE_2D_HPP
#define OPT_UTILS_STATE_2D_HPP

#include <list>
#include <vector>

#include "motion.hpp"
#include "pose2d.hpp"
#include "wrap2pi.hpp"

namespace hmpl {

class State2D : public hmpl::Pose2D {
 public:
    // PUBLIC ATTRIBUTES

    // the current gear
    Gear gear;

    // the desired speed
    double v;

    // the steering angle
    double phi;

    // the command time
    double t;

    // distance to the last cuscp
    double last_cusp_dist;

    // coming to stop flag
    bool coming_to_stop;

    // simple constructor
    State2D();

    // simple constructor, the input is a pose
    State2D(const Pose2D&, Gear g = ForwardGear, double vel = 0.0,
            double wheel_angle = 0.0, double t_ = 0.0, double lcd = 0,
            bool stop = false);

    // copy constructor
    State2D(const State2D&);

    // distance between two poses
    double distance(const State2D&);

    // distance squared between two poses
    double distance2(const State2D&);

    // get difference between orientations (yaw)
    double getOrientationDiff(const State2D&);

    // get difference between orientations (yaw), overloaded version
    double getOrientationDiff(double);

    // the assignment operator
    void operator=(const State2D&);

    // alternative assignement operator
    void operator=(const Pose2D&);

    // == operator overloading
    bool operator==(const State2D&);

    // != operator overloading
    bool operator!=(const State2D&);
};

// a helper class to avoid copies
class StateList {
 public:
    // EVERYTHING PUBLIC
    std::list<State2D> states;
};

typedef StateList* StateListPtr;

//
class StateArray {
 public:
    // EVERYTHING PUBLIC
    std::vector<State2D> states;
};

typedef StateArray* StateArrayPtr;

typedef StateArray& StateArrayRef;
}  // namespace hmpl

#endif  // OPT_UTILS_STATE_2D_HPP
