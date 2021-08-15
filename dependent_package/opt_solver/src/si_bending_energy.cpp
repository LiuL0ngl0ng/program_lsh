/* Copyright (c) 2016, Yu Zhang, Beijing Institute of Technology
 * All rights reserved.
 *        Files: SIBendingEnergy.cpp
 *   Created on: July, 31, 2016
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#include "opt_solver/si_bending_energy.hpp"
#include <cmath>

namespace hmpl {
SIBendingEnergy::SIBendingEnergy() {}

SIBendingEnergy::~SIBendingEnergy() {}

double SIBendingEnergy::getSIBendingEnergy(ParamKnot param) {
    double f;
    double p0;
    double p1;
    double p2;
    double p3;
    double p4;
    double t1;
    double t2;
    double t3;
    double t4;
    double t5;
    double t6;
    p0 = param.p0;
    p1 = param.p1;
    p2 = param.p2;
    p3 = param.p3;
    p4 = param.sf;
    t1 = 9;
    t2 = 27;
    t2 = -t1 * (p0 - p3) + t2 * (p1 - p2);
    t3 = -p3 * t1 + 18 * p0 - 45 * p1 + 36 * p2;
    t1 = p2 * t1 + 11 * p0 - 18 * p1 - 2 * p3;
    t4 = 0.1e1 / p4;
    t5 = pow(t4, 2);
    t6 = -t1 / 2;
    f = p4 * p4 *
        (((t4 * t5 * (p0 * t2 + t6 * t3) / 4 +
           pow(t5, 2) * ((double)pow((double)t3, (double)2) / 4 + t6 * t2) *
                   p4 / 5) *
                  p4 +
          t5 * (p0 * t3 + (double)pow((double)t1, (double)2) / 4) / 3) *
                 p4 * p4 +
         (t6 + p0) * p0 + t2 * (t2 / 28 + t3 / 12));
    return (f);
}

double SIBendingEnergy::dsiedp1(ParamKnot param) {
    double f;
    double p0;
    double p1;
    double p2;
    double p3;
    double p4;
    double t1;
    double t2;
    double t3;
    double t4;
    double t5;
    double t6;
    double t7;
    p0 = param.p0;
    p1 = param.p1;
    p2 = param.p2;
    p3 = param.p3;
    p4 = param.sf;
    t1 = 9;
    t2 = 27;
    t3 = -t1 * (p0 - p3) + t2 * (p1 - p2);
    t4 = -p3 * t1 + 18 * p0 - 45 * p1 + 36 * p2;
    t5 = p2 * t1 + 11 * p0 - 18 * p1 - 2 * p3;
    t6 = 0.1e1 / p4;
    t7 = pow(t6, 2);
    f = p4 * p4 * (p0 * t1 + 0.9e1 / 0.4e1 * t4 +
                   (((p0 * t2 + t1 * t4 + 0.45e2 / 0.2e1 * t5) * t6 * t7 / 4 +
                     (t1 * t3 - 0.27e2 / 0.2e1 * t5 - 0.45e2 / 0.2e1 * t4) *
                             pow(t7, 2) * p4 / 5) *
                            p4 +
                    t7 * (-t1 * t5 - 45 * p0) / 3) *
                           p4 * p4 -
                   0.51e2 / 0.28e2 * t3);
    return (f);
}

double SIBendingEnergy::dsiedp2(ParamKnot param) {
    double f;
    double p0;
    double p1;
    double p2;
    double p3;
    double p4;
    double t1;
    double t2;
    double t3;
    double t4;
    double t5;
    double t6;
    p0 = param.p0;
    p1 = param.p1;
    p2 = param.p2;
    p3 = param.p3;
    p4 = param.sf;
    t1 = 27;
    t2 = 9;
    t3 = -t2 * (p0 - p3) + t1 * (p1 - p2);
    t4 = -p3 * t2 + 18 * p0 - 45 * p1 + 36 * p2;
    t2 = p2 * t2 + 11 * p0 - 18 * p1 - 2 * p3;
    t5 = 0.1e1 / p4;
    t6 = pow(t5, 2);
    f = p4 * p4 * (0.15e2 / 0.14e2 * t3 - 0.9e1 / 0.4e1 * t4 +
                   ((-(p0 * t1 + 18 * t2 + 0.9e1 / 0.2e1 * t4) * t5 * t6 / 4 +
                     (0.27e2 / 0.2e1 * t2 - 0.9e1 / 0.2e1 * t3 + 18 * t4) *
                             pow(t6, 2) * p4 / 5) *
                            p4 +
                    t6 * (36 * p0 + 0.9e1 / 0.2e1 * t2) / 3) *
                           p4 * p4 -
                   0.9e1 / 0.2e1 * p0);
    return (f);
}

double SIBendingEnergy::dsiedp3(ParamKnot param) {
    double f;
    double p0;
    double p1;
    double p2;
    double p3;
    double p4;
    double t1;
    double t2;
    double t3;
    double t4;
    double t5;
    double t6;
    double t7;
    p0 = param.p0;
    p1 = param.p1;
    p2 = param.p2;
    p3 = param.p3;
    p4 = param.sf;
    t1 = 9;
    t2 = 27;
    t2 = -t1 * (p0 - p3) + t2 * (p1 - p2);
    t3 = -p3 * t1 + 18 * p0 - 45 * p1 + 36 * p2;
    t4 = p2 * t1 + 11 * p0 - 18 * p1 - 2 * p3;
    t5 = 0.1e1 / p4;
    t6 = pow(t5, 2);
    t7 = pow(t6, 2);
    t1 = t1 * p0;
    f = p4 * p4 * (0.3e1 / 0.4e1 * t3 +
                   (((t3 + 0.9e1 / 0.2e1 * t4 + t1) * t5 * t6 / 4 +
                     (t2 * t7 / 5 - 0.9e1 / 0.10e2 * t7 * (t3 + t4)) * p4) *
                            p4 -
                    t6 * (t4 + t1) / 3) *
                           p4 * p4 +
                   p0 - 0.3e1 / 0.28e2 * t2);
    return (f);
}

double SIBendingEnergy::dsiedp4(ParamKnot param) {
    double f;
    double p0;
    double p1;
    double p2;
    double p3;
    double p4;
    double t1;
    double t10;
    double t11;
    double t12;
    double t13;
    double t14;
    double t15;
    double t16;
    double t17;
    double t18;
    double t2;
    double t3;
    double t4;
    double t5;
    double t6;
    double t7;
    double t8;
    double t9;
    p0 = param.p0;
    p1 = param.p1;
    p2 = param.p2;
    p3 = param.p3;
    p4 = param.sf;
    t1 = 9;
    t2 = 27;
    t2 = -t1 * (p0 - p3) + t2 * (p1 - p2);
    t3 = -p3 * t1 + 18 * p0 - 45 * p1 + 36 * p2;
    t1 = p2 * t1 + 11 * p0 - 18 * p1 - 2 * p3;
    t4 = 0.1e1 / p4;
    t5 = -0.1e1 / 0.2e1;
    t6 = pow(t4, 2);
    t7 = pow(t6, 2);
    t8 = t4 * t6;
    t9 = (double)pow((double)t3, (double)2);
    t10 = t5 * t1;
    t11 = p0 * t2;
    t12 = t8 * (t10 * t3 + t11);
    t13 = (double)pow((double)t1, (double)2);
    t14 = p0 * t3;
    t6 = t6 * (t13 / 4 + t14);
    t15 = p0 * p0;
    t16 = p4 * p4;
    t17 = t7 * (t9 / 4 + t10 * t2) * p4;
    t18 = (t2 / 28 + t3 / 12) * t2;
    t10 = t10 * p0;
    f = p4 * (t16 * ((t12 / 4 + t17 / 5) * p4 + t6 / 3) + t10 + t18) +
        p4 * (2 * t15 +
              ((t7 * (-3 * t11 + 0.3e1 / 0.2e1 * t1 * t3) / 4 +
                t4 * t7 * (2 * t1 * t2 - t9) * p4 / 5) *
                       p4 +
               t8 * (t13 * t5 - 2 * t14) / 3) *
                      p4 * t16 +
              t16 * ((t12 + t17) * p4 + t6) + t18 + t10);
    return (f);
}

double SIBendingEnergy::dsied11(ParamKnot param) {
    double f;
    double p4;
    p4 = param.sf;
    f = 0.27e2 / 0.35e2 * p4 * p4;
    return (f);
}

double SIBendingEnergy::dsied21(ParamKnot param) {
    double f;
    double p4;
    p4 = param.sf;
    f = -0.27e2 / 0.280e3 * p4 * p4;
    return (f);
}

double SIBendingEnergy::dsied22(ParamKnot param) {
    double f;
    double p4;
    p4 = param.sf;
    f = 0.27e2 / 0.35e2 * p4 * p4;
    return (f);
}

double SIBendingEnergy::dsied31(ParamKnot param) {
    double f;
    double p4;
    p4 = param.sf;
    f = -0.3e1 / 0.70e2 * p4 * p4;
    return (f);
}

double SIBendingEnergy::dsied32(ParamKnot param) {
    double f;
    double p4;
    p4 = param.sf;
    f = 0.33e2 / 0.280e3 * p4 * p4;
    return (f);
}

double SIBendingEnergy::dsied33(ParamKnot param) {
    double f;
    double p4;
    p4 = param.sf;
    f = 0.16e2 / 0.105e3 * p4 * p4;
    return (f);
}

double SIBendingEnergy::dsied41(ParamKnot param) {
    double f;
    double p0;
    double p1;
    double p2;
    double p3;
    double p4;
    double t1;
    double t10;
    double t11;
    double t12;
    double t13;
    double t2;
    double t3;
    double t4;
    double t5;
    double t6;
    double t7;
    double t8;
    double t9;
    p0 = param.p0;
    p1 = param.p1;
    p2 = param.p2;
    p3 = param.p3;
    p4 = param.sf;
    t1 = 27;
    t2 = 9;
    t3 = -t2 * (p0 - p3) + t1 * (p1 - p2);
    t4 = -p3 * t2 + 18 * p0 - 45 * p1 + 36 * p2;
    t5 = p2 * t2 + 11 * p0 - 18 * p1 - 2 * p3;
    t6 = 0.1e1 / p4;
    t7 = pow(t6, 2);
    t8 = pow(t7, 2);
    t9 = t6 * t7;
    t10 = (p0 * t1 + t2 * t4 + 0.45e2 / 0.2e1 * t5) * t9;
    t7 = t7 * (-t2 * t5 - 45 * p0);
    t11 = -0.135e3 / 0.2e1;
    t12 = p4 * p4;
    t13 = (t2 * t3 - 0.27e2 / 0.2e1 * t5 - 0.45e2 / 0.2e1 * t4) * t8 * p4;
    f = (p0 * t2 + t12 * ((t10 / 4 + t13 / 5) * p4 + t7 / 3) -
         0.51e2 / 0.28e2 * t3 + 0.9e1 / 0.4e1 * t4 +
         (((-t1 * t4 + t11 * t5 - 81 * p0) * t8 / 4 +
           (-36 * t3 + 90 * t4 + 54 * t5) * t6 * t8 * p4 / 5) *
                  p4 +
          t9 * (90 * p0 + 18 * t5) / 3) *
                 p4 * t12 +
         0.1845e4 / 0.28e2 * p0 - 0.1053e4 / 0.7e1 * p1 +
         0.3645e4 / 0.28e2 * p2 - 0.513e3 / 0.14e2 * p3 +
         ((t10 + t13) * p4 + t7) * t12) *
        p4;
    return (f);
}

double SIBendingEnergy::dsied42(ParamKnot param) {
    double f;
    double p0;
    double p1;
    double p2;
    double p3;
    double p4;
    double t1;
    double t10;
    double t11;
    double t12;
    double t13;
    double t2;
    double t3;
    double t4;
    double t5;
    double t6;
    double t7;
    double t8;
    double t9;
    p0 = param.p0;
    p1 = param.p1;
    p2 = param.p2;
    p3 = param.p3;
    p4 = param.sf;
    t1 = 27;
    t2 = 9;
    t3 = -t2 * (p0 - p3) + t1 * (p1 - p2);
    t4 = -p3 * t2 + 18 * p0 - 45 * p1 + 36 * p2;
    t5 = p2 * t2 + 11 * p0 - 18 * p1 - 2 * p3;
    t6 = 0.1e1 / p4;
    t7 = pow(t6, 2);
    t8 = pow(t7, 2);
    t9 = t6 * t7;
    t1 = -(p0 * t1 + 0.9e1 / 0.2e1 * t4 + 18 * t5) * t9;
    t7 = t7 * (36 * p0 + 0.9e1 / 0.2e1 * t5);
    t10 = -72;
    t11 = 54 * t5;
    t12 = p4 * p4;
    t13 = (0.27e2 / 0.2e1 * t5 - 0.9e1 / 0.2e1 * t3 + 18 * t4) * t8 * p4;
    f = (0.15e2 / 0.14e2 * t3 - 0.9e1 / 0.4e1 * t4 +
         ((t1 / 4 + t13 / 5) * p4 + t7 / 3) * t12 - 0.414e3 / 0.7e1 * p0 +
         (((81 * p0 + 0.27e2 / 0.2e1 * t4 + t11) * t8 / 4 +
           (t10 * t4 - t11 + 18 * t3) * t6 * t8 * p4 / 5) *
                  p4 +
          t9 * (p0 * t10 - t2 * t5) / 3) *
                 p4 * t12 +
         0.3645e4 / 0.28e2 * p1 - 0.1539e4 / 0.14e2 * p2 +
         0.837e3 / 0.28e2 * p3 + ((t1 + t13) * p4 + t7) * t12) *
        p4;
    return (f);
}

double SIBendingEnergy::dsied43(ParamKnot param) {
    double f;
    double p0;
    double p1;
    double p2;
    double p3;
    double p4;
    double t1;
    double t10;
    double t11;
    double t12;
    double t13;
    double t14;
    double t2;
    double t3;
    double t4;
    double t5;
    double t6;
    double t7;
    double t8;
    double t9;
    p0 = param.p0;
    p1 = param.p1;
    p2 = param.p2;
    p3 = param.p3;
    p4 = param.sf;
    t1 = 27;
    t2 = 9;
    t3 = -t2 * (p0 - p3) + t1 * (p1 - p2);
    t4 = 18 * p0;
    t5 = -p3 * t2 - 45 * p1 + 36 * p2 + t4;
    t6 = p2 * t2 + 11 * p0 - 18 * p1 - 2 * p3;
    t7 = 0.1e1 / p4;
    t8 = t5 + t6;
    t9 = pow(t7, 2);
    t10 = pow(t9, 2);
    t11 = t7 * t10;
    t7 = t7 * t9;
    t2 = t2 * p0;
    t12 = (t5 + 0.9e1 / 0.2e1 * t6 + t2) * t7;
    t2 = -t9 * (t6 + t2);
    t9 = -0.27e2 / 0.2e1;
    t13 = p4 * p4;
    t14 = (t10 * t3 - 0.9e1 / 0.2e1 * t10 * t8) * p4;
    f = (t13 * ((t12 / 4 + t14 / 5) * p4 + t2 / 3) + 0.3e1 / 0.4e1 * t5 -
         0.3e1 / 0.28e2 * t3) *
                p4 +
        p4 * (0.461e3 / 0.28e2 * p0 +
              (((-p0 * t1 + t6 * t9 - 3 * t5) * t10 / 4 +
                (-0.4e1 / 0.5e1 * t11 * t3 + 0.18e2 / 0.5e1 * t11 * t8) * p4) *
                       p4 +
               t7 * (t4 + 2 * t6) / 3) *
                      p4 * t13 -
              0.513e3 / 0.14e2 * p1 + 0.837e3 / 0.28e2 * p2 -
              0.54e2 / 0.7e1 * p3 + ((t14 + t12) * p4 + t2) * t13);
    return (f);
}

double SIBendingEnergy::dsied44(ParamKnot param) {
    double f;
    double p0;
    double p1;
    double p2;
    double p3;
    double p4;
    double t1;
    double t10;
    double t11;
    double t12;
    double t13;
    double t14;
    double t15;
    double t16;
    double t17;
    double t18;
    double t2;
    double t3;
    double t4;
    double t5;
    double t6;
    double t7;
    double t8;
    double t9;
    p0 = param.p0;
    p1 = param.p1;
    p2 = param.p2;
    p3 = param.p3;
    p4 = param.sf;
    t1 = 9;
    t2 = 27;
    t2 = -t1 * (p0 - p3) + t2 * (p1 - p2);
    t3 = -p3 * t1 + 18 * p0 - 45 * p1 + 36 * p2;
    t1 = p2 * t1 + 11 * p0 - 18 * p1 - 2 * p3;
    t4 = 0.1e1 / p4;
    t5 = (double)pow((double)t3, (double)2);
    t6 = pow(t4, 2);
    t7 = pow(t6, 2);
    t8 = t4 * t7;
    t4 = t4 * t6;
    t9 = 0.1e1 / 0.4e1;
    t10 = t1 / 2;
    t11 = t7 * (-t10 * t2 + t5 * t9);
    t12 = p0 * t2;
    t13 = t7 * (-3 * t12 + 0.3e1 / 0.2e1 * t1 * t3);
    t10 = t4 * (-t10 * t3 + t12);
    t14 = (double)pow((double)t1, (double)2);
    t15 = p0 * t3;
    t4 = -t4 * (t14 / 2 + 2 * t15);
    t16 = t6 * (t14 * t9 + t15);
    t17 = t8 * (2 * t1 * t2 - t5) * p4;
    t18 = p4 * p4;
    f = ((t13 / 2 + 0.2e1 / 0.5e1 * t17) * p4 + 0.2e1 / 0.3e1 * t4) * p4 * t18 +
        t2 * (t2 / 14 + t3 / 6) + 2 * p0 * p0 +
        2 * t18 * ((p4 * t11 + t10) * p4 + t16) - p0 * t1 +
        p4 * ((((t9 * t8 * (-6 * t1 * t3 + 12 * t12) +
                 t6 * t7 * (-10 * t1 * t2 + 5 * t5) * p4 / 5) *
                        p4 +
                4 * t11 + t7 * (0.3e1 / 0.2e1 * t14 + 6 * t15) / 3) *
                       p4 +
               3 * t10) *
                      t18 +
              2 * (((t17 + t13) * p4 + t4) * p4 + t16) * p4);
    return (f);
}

}  // namespace hmpl
