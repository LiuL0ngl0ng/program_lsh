/* Copyright (c) 2016, Yu Zhang, Beijing Institute of Technology
 * All rights reserved.
 *        Files: SE2Constraints.h
 *   Created on: July, 31, 2016
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#ifndef OPT_SOLVER_SE2CONSTRAINTS_H
#define OPT_SOLVER_SE2CONSTRAINTS_H

#include <opt_utils/utils.hpp>

//! Genereted codes ,  the formalation can be found in Kelly, Alonzo, and
//! Bryan Nagy. "Reactive nonholonomic trajectory generation via parametric
//! optimal control." The International Journal of Robotics Research
//! 22.7-8 (2003): 583-601.

/*!
 * SE2 constraints in terms of parameters.
 * @param[in] param : knot parameter
 * @return function values.
 */
namespace hmpl {
class SE2Constraints {
    SE2Constraints();
    ~SE2Constraints();

 public:
    static double fxf(ParamKnot param);  // x
    static double fyf(ParamKnot param);  // y
    static double ftf(ParamKnot param);  // theta  the heading of the car

    //! constrained jacobian of final constraints

    static double dxdp1(ParamKnot param);
    static double dxdp2(ParamKnot param);
    static double dxdp3(ParamKnot param);
    static double dxdp4(ParamKnot param);

    static double dydp1(ParamKnot param);
    static double dydp2(ParamKnot param);
    static double dydp3(ParamKnot param);
    static double dydp4(ParamKnot param);

    static double dtdp1(ParamKnot param);
    static double dtdp2(ParamKnot param);
    static double dtdp3(ParamKnot param);
    static double dtdp4(ParamKnot param);

    //! constrained hessian of final constraints
    static double dxd11(ParamKnot param);
    static double dxd21(ParamKnot param);
    static double dxd22(ParamKnot param);
    static double dxd31(ParamKnot param);
    static double dxd32(ParamKnot param);
    static double dxd33(ParamKnot param);
    static double dxd41(ParamKnot param);
    static double dxd42(ParamKnot param);
    static double dxd43(ParamKnot param);
    static double dxd44(ParamKnot param);

    static double dyd11(ParamKnot param);
    static double dyd21(ParamKnot param);
    static double dyd22(ParamKnot param);
    static double dyd31(ParamKnot param);
    static double dyd32(ParamKnot param);
    static double dyd33(ParamKnot param);
    static double dyd41(ParamKnot param);
    static double dyd42(ParamKnot param);
    static double dyd43(ParamKnot param);
    static double dyd44(ParamKnot param);

    static double dtd11(ParamKnot param);
    static double dtd21(ParamKnot param);
    static double dtd22(ParamKnot param);
    static double dtd31(ParamKnot param);
    static double dtd32(ParamKnot param);
    static double dtd33(ParamKnot param);
    static double dtd41(ParamKnot param);
    static double dtd42(ParamKnot param);
    static double dtd43(ParamKnot param);
    static double dtd44(ParamKnot param);
};
}  // namespace hmpl
#endif  // LOCAL_PLANNER_SE2CONSTRAINTS_H
