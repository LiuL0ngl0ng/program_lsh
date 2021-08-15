/* Copyright (c) 2017, Yu Zhang, Intelligent Vehicle Research Center(IVRC),
 * Beijing Institute of Technology.
 * All rights reserved.
 *        Files: solution.hpp
 *   Created on: 05, 04, 2017
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#ifndef OPT_SOLVER_SOLUTION_HPP
#define OPT_SOLVER_SOLUTION_HPP
#include <opt_utils/utils.hpp>

namespace hmpl {
class Solution {
 public:
    Solution();

 private:
    // origin of the local frame in global frame
    State origin_;
    // goal in global frame
    State goal_;
    // knot vecotr
    ParamKnot knots_;  // p0 p1 p2 p3 sf
};
}  // namespace hmpl

#endif  // OPT_SOLVER_SOLUTION_HPP
