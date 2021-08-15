/* Copyright (c) 2017, Yu Zhang, Intelligent Vehicle Research Center(IVRC),
 * Beijing Institute of Technology.
 * All rights reserved.
 *        Files: base.hpp
 *   Created on: 28, 09, 2017
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */

#ifndef OPT_UTILS_OPT_BASE_H
#define OPT_UTILS_OPT_BASE_H

#include <iostream>
#include <vector>

#include "utils.hpp"
#include "vector2d.hpp"
#include "circle.hpp"

// default sampling number

namespace hmpl {
/*!
 * @brief Convert knot parameters to corresponding coeffcient paramters
 * @param[in] param Knot parameters
 * @return  The coefficient parameters.
 */
ParamCoefficient knotToCoefficient(ParamKnot param);
/*!
 * @brief Sample the cubic spiral in terms of coefficient parameters
 * @param[in] start Initial state
 * @param[in] param  Coefficient parameters
 * @param[in] num  The sampling number
 * @return State vector
 */
std::vector<State> sampleCubicSpiral(State start, ParamCoefficient param,
                                     int num);

/*!
 * @brief Sample the cubic spiral in terms of coefficient parameters by step
 * length.
 * @param[in] start Start state
 * @param[in] param Parameter in coefficient form.
 * @param[in] stepLength Step length in meters.
 * @return Path vector of States.
 */
std::vector<State> sampleCubicSpiralByLength(State start,
                                             ParamCoefficient param,
                                             double stepLength);
/*!
 * @brief Calculate the function value of the heading.
 * @param[in] param Coefficient parameters
 * @param[in] s Arc length, start from 0, end at sf
 * @return The heading value, can't be wrapped.
 * Because it's a continuous function.
 */
double fTheta(ParamCoefficient param, double s);
/*!
 * @brief Calculate the function value of the curvature.
 * @param[in] param Coefficient parameters
 * @param[in] s Arc length, start from 0, end at sf
 * @return The curvature value at s arc length
 */
double fKappa(ParamCoefficient param, double s);

/**!
 * Get the distance of the two states
 * @param st1
 * @param st2
 * @return
 */
double getNorm(const State &st1, const State &st2);

/**!
 * Get the distance of the state
 * @param state
 * @return L2 norm
 */
double getNorm(const State &state);
/**
 * global to local frame
 * @param relTo origin point in global frame
 * @param target The target point
 * @return coordinate in the reference frame.
 */
State globalToLocal(const State &relTo, const State &target);

/**
 *  local to global frame
 * @param relTo
 * @param target
 * @return
 */
State localToGlobal(const State &relTo, const State &target);

/**
 * fill the arclength, heading and curvature information
 * @param path State path with xy information only
 */
void fillStatePath(std::vector<hmpl::State> *path);

}

#endif  // OPT_UTILS_OPT_BASE_H
