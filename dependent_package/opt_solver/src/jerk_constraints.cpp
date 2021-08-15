#include "opt_solver/jerk_constraints.hpp"

namespace hmpl {

double fjf(ParamKnot param) {
    double f;
    double p0;
    double p1;
    double p2;
    double p3;
    double p4;
    p0 = param.p0;
    p1 = param.p1;
    p2 = param.p2;
    p3 = param.p3;
    p4 = param.sf;
    f = (-p0 + 0.9e1 / 0.2e1 * p1 - 9 * p2 + 0.11e2 / 0.2e1 * p3) / p4;
    return (f);
}

double djdp1(ParamKnot param) {
    double f;
    double p0;
    double p1;
    double p2;
    double p3;
    double p4;
    p0 = param.p0;
    p1 = param.p1;
    p2 = param.p2;
    p3 = param.p3;
    p4 = param.sf;
    f = 0.9e1 / 0.2e1 / p4;
    return (f);
}

double djdp2(ParamKnot param) {
    double f;
    double p0;
    double p1;
    double p2;
    double p3;
    double p4;
    p0 = param.p0;
    p1 = param.p1;
    p2 = param.p2;
    p3 = param.p3;
    p4 = param.sf;
    f = -9 / p4;
    return (f);
}

double djdp4(ParamKnot param) {
    double f;
    double p0;
    double p1;
    double p2;
    double p3;
    double p4;
    p0 = param.p0;
    p1 = param.p1;
    p2 = param.p2;
    p3 = param.p3;
    p4 = param.sf;
    f = (p0 - 0.9e1 / 0.2e1 * p1 + 9 * p2 - 0.11e2 / 0.2e1 * p3) * pow(p4, -2);
    return (f);
}

double djd11(ParamKnot param) {
    double f;
    double p0;
    double p1;
    double p2;
    double p3;
    double p4;
    p0 = param.p0;
    p1 = param.p1;
    p2 = param.p2;
    p3 = param.p3;
    p4 = param.sf;
    f = 0;
    return (f);
}

double djd21(ParamKnot param) {
    double f;
    double p0;
    double p1;
    double p2;
    double p3;
    double p4;
    p0 = param.p0;
    p1 = param.p1;
    p2 = param.p2;
    p3 = param.p3;
    p4 = param.sf;
    f = 0;
    return (f);
}

double djd22(ParamKnot param) {
    double f;
    double p0;
    double p1;
    double p2;
    double p3;
    double p4;
    p0 = param.p0;
    p1 = param.p1;
    p2 = param.p2;
    p3 = param.p3;
    p4 = param.sf;
    f = 0;
    return (f);
}

double djd41(ParamKnot param) {
    double f;
    double p0;
    double p1;
    double p2;
    double p3;
    double p4;
    p0 = param.p0;
    p1 = param.p1;
    p2 = param.p2;
    p3 = param.p3;
    p4 = param.sf;
    f = -0.9e1 / 0.2e1 * pow(p4, -2);
    return (f);
}

double djd42(ParamKnot param) {
    double f;
    double p0;
    double p1;
    double p2;
    double p3;
    double p4;
    p0 = param.p0;
    p1 = param.p1;
    p2 = param.p2;
    p3 = param.p3;
    p4 = param.sf;
    f = 9 * pow(p4, -2);
    return (f);
}

double djd44(ParamKnot param) {
    double f;
    double p0;
    double p1;
    double p2;
    double p3;
    double p4;
    double t1;
    double t2;
    p0 = param.p0;
    p1 = param.p1;
    p2 = param.p2;
    p3 = param.p3;
    p4 = param.sf;
    t1 = 0.1e1 / p4;
    t2 = pow(t1, 2);
    f = (-2 * p0 + 9 * p1 - 18 * p2 + 11 * p3) * t1 * t2;
    return (f);
}

double fif(ParamKnot param) {
    double f = -0.5 *
               (11 * param.p0 - 18 * param.p1 + 9 * param.p2 - 2 * param.p3) /
               param.sf;
    return f;
}
double didp1(ParamKnot param) { return 9.0 / param.sf; }
double didp2(ParamKnot param) { return -9.0 / (2.0 * param.sf); }
double didp4(ParamKnot param) {
    double f = 0.5 *
               (11 * param.p0 - 18 * param.p1 + 9 * param.p2 - 2 * param.p3) /
               pow(param.sf, 2.0);
    return f;
}
double didp11(ParamKnot param) { return 0; }
double didp21(ParamKnot param) { return 0; }
double didp22(ParamKnot param) { return 0; }
double didp41(ParamKnot param) { return -9.0 / pow(param.sf, 2.0); }
double didp42(ParamKnot param) { return 9.0 / (2 * pow(param.sf, 2.0)); }

double didp44(ParamKnot param) {
    double f = -(11 * param.p0 - 18 * param.p1 + 9 * param.p2 - 2 * param.p3) /
               pow(param.sf, 3.0);
}
}
