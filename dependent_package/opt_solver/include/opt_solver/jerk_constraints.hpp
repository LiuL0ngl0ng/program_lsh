#ifndef OPT_SOLVER_JERK_CONSTRAINTS_H
#define OPT_SOLVER_JERK_CONSTRAINTS_H
#include <opt_utils/opt_utils.hpp>
namespace hmpl {

double fjf(ParamKnot param);
double djdp1(ParamKnot param);
double djdp2(ParamKnot param);
double djdp4(ParamKnot param);

double djd11(ParamKnot param);
double djd21(ParamKnot param);
double djd22(ParamKnot param);

double djd41(ParamKnot param);
double djd42(ParamKnot param);
double djd44(ParamKnot param);

double fif(ParamKnot param);
double didp1(ParamKnot param);
double didp2(ParamKnot param);
double didp4(ParamKnot param);
double didp11(ParamKnot param);
double didp21(ParamKnot param);
double didp22(ParamKnot param);
double didp41(ParamKnot param);
double didp42(ParamKnot param);
double didp44(ParamKnot param);

}  // namespace hmpl

#endif  // OPT_SOLVER_JERK_CONSTRAINTS_H