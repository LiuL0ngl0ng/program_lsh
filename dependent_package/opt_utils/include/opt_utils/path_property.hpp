//
// Created by yangt on 19-5-2.
//

#ifndef OPT_UTILS_PATH_PROPERTY_HPP
#define OPT_UTILS_PATH_PROPERTY_HPP

#include <vector>
#include <geometry_msgs/Pose.h>
#include <lanelet_map_msgs/Node.h>
#include <glog/logging.h>

#include "circle.hpp"
#include "vector2d.hpp"
#include "constants.hpp"

namespace hmpl {

template<class PointType>
double &xRef(PointType &point) {
    return point.x;
}

template<class PointType>
const double &xRef(const PointType &point) {
    return point.x;
}

template<class PointType>
double &yRef(PointType &point) {
    return point.y;
}

template<class PointType>
const double &yRef(const PointType &point) {
    return point.y;
}

template<>
double &xRef(hmpl::Circle &point);

template<>
double &xRef(Vector2D<double> &point);

template<>
const double &xRef(const hmpl::Circle &point);

template<>
const double &xRef(const Vector2D<double> &point);

template<>
double &yRef(hmpl::Circle &point);

template<>
double &yRef(Vector2D<double> &point);

template<>
const double &yRef(const hmpl::Circle &point);

template<>
const double &yRef(const Vector2D<double> &point);

template<>
double &xRef(lanelet_map_msgs::Node &point);

template<>
double &xRef(geometry_msgs::Pose &point);

template<>
const double &xRef(const lanelet_map_msgs::Node &point);

template<>
const double &xRef(const geometry_msgs::Pose &point);

template<>
double &yRef(lanelet_map_msgs::Node &point);

template<>
double &yRef(geometry_msgs::Pose &point);

template<>
const double &yRef(const lanelet_map_msgs::Node &point);

template<>
const double &yRef(const geometry_msgs::Pose &point);

template<class PointType>
double distance(const PointType &p1, const PointType &p2) {
    return sqrt(pow(xRef(p1) - xRef(p2), 2) + pow(yRef(p1) - yRef(p2), 2));
}

template<class PointType>
double angle(const PointType &p1, const PointType &p2) {
    return atan2(yRef(p2) - yRef(p1), xRef(p2) - xRef(p1));
}

/*!
 * @brief Calculate the curvature of a circle base on three points
 * The math behind this function is
 * http://mathworld.wolfram.com/SSSTheorem.html
 * @param first_point The first point.
 * @param second_point The second point.
 * @param third_point The third point.
 * @return curvature The curvature of the circle determined by the three
 * points. According to ros coordinates convention, the left is positive and
 * the right is negative for curvature.
 */
template<class PointType>
double getCurvature(const PointType &first_point,
                    const PointType &second_point,
                    const PointType &third_point) {
    // caculate the radius of the circle first
    double a, b, c;  // three sides
    double delta_x, delta_y;
    double s;  // semiperimeter
    double K;

    // side one
    a = distance(first_point, second_point);

    // side two
    b = distance(second_point, third_point);

    // side three
    c = distance(first_point, third_point);

    if (a < CLOSE_TO_ZERO || b < CLOSE_TO_ZERO || c < CLOSE_TO_ZERO) {
        return 0;
    }

    // semiperimeter
    s = (a + b + c) / 2.0;
    K = sqrt(fabs(s * (s - a) * (s - b) * (s - c)));
    double curvature = 4 * K / (a * b * c);


    // determine the sign
    double rotate_direction =
        (xRef(second_point) - xRef(first_point)) *
            (yRef(third_point) - yRef(second_point)) -
            (yRef(second_point) - yRef(first_point))
                * (xRef(third_point) - xRef(second_point));
    if (rotate_direction < 0) {
        // clock wise
        curvature = -curvature;
    }  // otherwise counter clockwise
    return curvature;
}

template<class PointType>
double getSmoothness(const PointType &first,
                     const PointType &second,
                     const PointType &third) {
    return getCurvature(first, second, third);
}

template<class PointType>
double getOMPLSmoothness(const PointType &first,
                         const PointType &second,
                         const PointType &third) {
    const double a = distance(first, second);
    const double b = distance(second, third);
    const double c = distance(first, third);

    double f = 0;

    double acos_value = (a * a + b * b - c * c) / (2 * a * b);

    if (acos_value >= -1.0 && acos_value <= 1.0) {
        // the smoothness is actually the outside angle of the one we compute
        double angle = M_PI - acos(acos_value);
        // normolize by length of the segments
        double k = 2.0 * angle / (a + b);
        f += k * k;
    }
    return f;
}

template<class PointType>
double getPathSmoothness(const std::vector<PointType> &path, bool ompl) {
    std::size_t size = path.size();
    if (size < 3) {
        LOG(INFO) << "The size of the circle path points is below 3. "
            "Can't get smoothness.";
        return 0;
    }

    double f = 0;
    double smoothness = 0;
    for (std::size_t i = 1; i < size - 2; i++) {
        const PointType &first = path.at(i - 1);
        const PointType &second = path.at(i);
        const PointType &third = path.at(i + 1);
        if (ompl) {
            smoothness = getOMPLSmoothness(first, second, third);
        } else {
            smoothness = getSmoothness(first, second, third);
        }
        f = f + pow(smoothness, 2.0);
    }
    return f;
}

template<class PointType>
double getLengthOfPath(const std::vector<PointType> &path) {
    std::size_t size = path.size();
    double length = 0;
    if (size >= 2) {
        for (std::size_t i = 0; i < size - 2; i++) {
            length += distance(path.at(i), path.at(i + 1));
        }
        return length;
    } else {
        LOG(INFO) << "path size is less than 2";
        return length;
    }
}
}
#endif //OPT_UTILS_PATH_PROPERTY_HPP
