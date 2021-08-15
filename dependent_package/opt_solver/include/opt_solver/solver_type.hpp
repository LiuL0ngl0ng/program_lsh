/* Copyright (c) 2017, Yu Zhang, Intelligent Vehicle Research Center(IVRC),
 * Beijing Institute of Technology.
 * All rights reserved.
 *        Files: solver_type.hpp
 *   Created on: 05, 04, 2017
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#ifndef OPT_SOLVER_SOLVER_TYPE_HPP
#define OPT_SOLVER_SOLVER_TYPE_HPP
namespace hmpl {

enum class SolverType {
    CERES = 0,
    IPOPT = 1,
    SNOPT = 2,
    NLOPT = 3,
    RANDOM = 4
};

}  // namespace hmpl

#endif  // OPT_SOLVER_SOLVER_TYPE_HPP
