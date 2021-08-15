#include "opt_utils/base.hpp"
#include <cmath>
#include <vector>
#include "opt_utils/constants.hpp"
#include "opt_utils/path_property.hpp"

namespace hmpl {

ParamCoefficient knotToCoefficient(ParamKnot param) {
    ParamCoefficient pv;
    double s = param.sf;
    pv.sf = param.sf;
    pv.a = param.p0;

    pv.b = (18 * param.p1 + 2 * param.p3 - 11 * param.p0 - 9 * param.p2) /
            (2 * s);
    pv.c = (18 * param.p0 - 45 * param.p1 + 36 * param.p2 - 9 * param.p3) /
            (2 * pow(s, 2));
    pv.d = (27 * param.p1 - 27 * param.p2 + 9 * param.p3 - 9 * param.p0) /
            (2 * pow(s, 3));
    return pv;
}

double fTheta(ParamCoefficient param, double s) {
    double s2 = pow(s, 2);
    double s3 = s2 * s;
    double s4 = s3 * s;
    double f = param.a * s + param.b * s2 / 2 + param.c * s3 / 3 +
            param.d * s4 / 4;
    return f;
}

double fKappa(ParamCoefficient param, double s) {
    double f = param.a + param.b * s + param.c * s * s + param.d * s * s * s;
    return f;
}

// trapezoid rule to calculate the fresnel intergral
std::vector<State> sampleCubicSpiral(State start, ParamCoefficient param,
                                     int num) {
    State pose;
    start.s = 0;
    start.v = 0;

    std::vector<State> path;
    path.clear();
    path.push_back(start);
    double deltax = 0;
    double deltay = 0;
    double deltax1 = 0;
    double deltay1 = 0;
    double theta = 0;
    double theta1 = 0;
    double s = 0;
    for (int i = 1; i <= num; i++) {
        s = i * param.sf / num;
        theta = fTheta(param, s);

        if (i == 1) {
            deltax = (cos(theta) + cos(0)) / (2 * i);
            deltay = (sin(theta) + sin(0)) / (2 * i);
        } else {
            deltax = (cos(theta) + cos(theta1)) / (2 * i) +
                    deltax1 * (i - 1) / i;
            deltay = (sin(theta) + sin(theta1)) / (2 * i) +
                    deltay1 * (i - 1) / i;
        }
        pose.x = deltax * s;
        pose.y = deltay * s;
        pose.z = theta;
        pose.k = fKappa(param, s);
        pose.s = s;
        pose.v = 0.0;
        path.push_back(pose);
        deltax1 = deltax;
        deltay1 = deltay;
        theta1 = theta;
    }
    return path;
}

std::vector<State> sampleCubicSpiralByLength(State start,
                                             ParamCoefficient param,
                                             double stepLength) {
    State pose;
    start.s = 0;
    start.v = 0;
    // number of segments
    int num = static_cast<int>(floor(param.sf / stepLength));

    std::vector<State> path;
    path.clear();
    path.push_back(start);
    double deltax = 0;
    double deltay = 0;
    double deltax1 = 0;
    double deltay1 = 0;
    double theta = 0;
    double theta1 = 0;
    double s = 0;
    for (int i = 1; i <= num; i++) {
        s = i * param.sf / num;
        // get the corresponding heading angle according to arc-length along
        // the path and the parameters of the spiral
        theta = fTheta(param, s);

        if (i == 1) {
            deltax = (cos(theta) + cos(0)) / (2 * i);
            deltay = (sin(theta) + sin(0)) / (2 * i);
        } else {
            deltax = (cos(theta) + cos(theta1)) / (2 * i) +
                    deltax1 * (i - 1) / i;
            deltay = (sin(theta) + sin(theta1)) / (2 * i) +
                    deltay1 * (i - 1) / i;
        }
        pose.x = deltax * s;
        pose.y = deltay * s;
        pose.z = theta;
        pose.k = fKappa(param, s);
        pose.s = s;
        pose.v = 0.0;
        path.push_back(pose);
        deltax1 = deltax;
        deltay1 = deltay;
        theta1 = theta;
    }
    return path;
}

double getNorm(const State &st1, const State &st2) {
    double dx = st1.x - st2.x;
    double dy = st1.y - st2.y;
    return sqrt(dx * dx + dy * dy);
}

double getNorm(const State &state) {
    double x = state.x;
    double y = state.y;
    return sqrt(x * x + y * y);
}

State globalToLocal(const State &relTo, const State &target) {
    State local_target_state{};
    double deltaX = target.x - relTo.x;
    double deltaY = target.y - relTo.y;
    double deltaT = target.z - relTo.z;
    local_target_state.x = deltaX * cos(relTo.z) + deltaY * sin(relTo.z);
    local_target_state.y = -deltaX * sin(relTo.z) + deltaY * cos(relTo.z);
    local_target_state.z = deltaT;
    local_target_state.k = target.k;
    local_target_state.dk = target.dk;
    local_target_state.s = 0;
    local_target_state.v = target.v;
    return local_target_state;
}

State localToGlobal(const State &relTo, const State &target) {
    State global_target_state{};
    global_target_state.x =
            (target.x * cos(relTo.z) - target.y * sin(relTo.z)) + relTo.x;
    global_target_state.y =
            (target.x * sin(relTo.z) + target.y * cos(relTo.z)) + relTo.y;
    global_target_state.z = relTo.z + target.z;
    global_target_state.k = target.k;
    global_target_state.v = target.v;
    global_target_state.s = target.s;
    global_target_state.dk = target.dk;
    return global_target_state;
}

void fillStatePath(std::vector<hmpl::State> *path) {
    if (path->empty()) {
        return;
    }
    std::size_t size = path->size();
    double s = 0;
    for (std::size_t i = 0; i < size - 1; i++) {
        if (i == 0) {
            path->at(i).k = 0;
            path->at(i).s = 0;
        }
        hmpl::State &state1 = path->at(i);
        hmpl::State &state2 = path->at(i + 1);
        double dx = state2.x - state1.x;
        double dy = state2.y - state1.y;
        double delta_s = sqrt(dx * dx + dy * dy);
        path->at(i).z = atan2(dy, dx);
        s += delta_s;
        path->at(i + 1).s = s;
        if (i + 2 < size - 2) {
            path->at(i + 1).k =
                    hmpl::getCurvature(state1, state2, path->at(i + 2));
        }
    }
    path->back().z = path->at(size - 2).z;
    path->back().k = 0;
}

}  // namespace hmpl
