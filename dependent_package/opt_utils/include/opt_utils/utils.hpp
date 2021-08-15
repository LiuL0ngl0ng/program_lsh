/* Copyright (c) 2017, Yu Zhang, Intelligent Vehicle Research Center(IVRC),
 * Beijing Institute of Technology.
 *        Files: time.hpp
 *   Created on: 09, 11, 2017
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#ifndef OPT_UTILS_UTILS_HPP
#define OPT_UTILS_UTILS_HPP

#include <vector>
#include "time.hpp"
namespace hmpl {
/*!
 * @brief Coefficients parameters
 *
 * \f$ \kappa(s) = a + b*s + c*s^2 + d*s^3 , s \in [0,sf]\f$
 */
struct ParamCoefficient {
    double a;
    double b;
    double c;
    double d;
    double sf;
};

/*!
 * @brief knot parameters
 * \p p0 initial curvature
 * \p p1 curvature at 1/3 of \p sf
 * \p p2 curvature at 2/3 of \p sf
 * \p p3 curvature at \p sf
 */
struct ParamKnot {
    double p0;  // initial curvature
    double p1;  // curvature in 1/3
    double p2;  // curvature in 2/3
    double p3;  // curvature in sf
    double sf;  // sf -> p4
};

/*!
 * @brief Vehicle state struct
 * \p x x position
 * \p y y position
 * \p z heading of the car
 * \p k curvature at this state
 * \p v speed at this state
 * \p s arc length at this state
 * todo: create a class for state
 */
struct State {
    double x{};
    double y{};
    double z{};
    double k{};
    double s{};
    double v{};
    double dk{};
};

struct PathWithCost {
    State origin{};         // the origin of the path in the global frame
    State goal{};           // goal state of the path win the global frame
    ParamKnot param{};      // knot parameter
    double cost{};          // final cost consider everything.
    double goal_cost{};     // normilized distance cost to the goal on the path
    double bending_cost{};  // measurement of bending energy of the path
    bool feasible{true};    // determined by maximum curvature
    bool collision_free{true};  // collision checking
    bool truncated{};
    std::vector<State>
            path{};  // path vector for collision checking by resolution
};

/*!
 * @brief simple 2d point data
 */
struct Point {
    double x{};
    double y{};
};

struct Center {
    double x{};  //  the x coordinates of the center of the circle
    double y{};  //  the y coordinates of the center of the circle
    double r{};  //  radius of the circle
};

}  // namespace hmpl

#endif  // OPT_UTILS_UTILS_HPP
