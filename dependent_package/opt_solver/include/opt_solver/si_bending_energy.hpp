/* Copyright (c) 2016, Yu Zhang, Beijing Institute of Technology
 * All rights reserved.
 *        Files: si_bending_energy.h
 *   Created on: July, 31, 2016
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#ifndef OPT_SOLVER_SI_BENDING_ENERGY_H
#define OPT_SOLVER_SI_BENDING_ENERGY_H

#include <opt_utils/utils.hpp>

namespace hmpl {
/**
 * This class can calculate function value, gradient and hessain of bending
 * energy of the curve in terms of parameters.
 *
 * Please refer to
 * Moreton, Henry P., and Carlo H. Séquin. "Scale‐Invariant Minimum‐Cost Curves:
 * Fair and Robust Design Implements." Computer Graphics Forum. Vol. 12. No.
 * 3. Blackwell Science Ltd, 1993 for details.
 */
class SIBendingEnergy {
 public:
    // constructor
    SIBendingEnergy();
    ~SIBendingEnergy();

    //! function evaluation method
    static double getSIBendingEnergy(ParamKnot param);

    //! gradient

    static double dsiedp1(ParamKnot param);

    static double dsiedp2(ParamKnot param);

    static double dsiedp3(ParamKnot param);

    static double dsiedp4(ParamKnot param);

    //! hessian matrix

    static double dsied11(ParamKnot param);

    static double dsied21(ParamKnot param);

    static double dsied22(ParamKnot param);

    static double dsied31(ParamKnot param);

    static double dsied32(ParamKnot param);

    static double dsied33(ParamKnot param);

    static double dsied41(ParamKnot param);

    static double dsied42(ParamKnot param);

    static double dsied43(ParamKnot param);

    static double dsied44(ParamKnot param);
};

}  // namespace hmpl
#endif  // OPT_SOLVER_SI_BENDING_ENERGY_H
