//
// Created by yangt on 19-5-3.
//

#include "opt_utils/path_property.hpp"
namespace hmpl {

template<>
double &xRef(hmpl::Circle &point) {
    return point.position.x;
}

template<>
double &xRef(Vector2D<double> &point) {
    return point.x;
}

template<>
const double &xRef(const hmpl::Circle &point) {
    return point.position.x;
}

template<>
const double &xRef(const Vector2D<double> &point) {
    return point.x;
}

template<>
double &yRef(hmpl::Circle &point) {
    return point.position.y;
}

template<>
double &yRef(Vector2D<double> &point) {
    return point.y;
}

template<>
const double &yRef(const hmpl::Circle &point) {
    return point.position.y;
}

template<>
const double &yRef(const Vector2D<double> &point) {
    return point.y;
}

template<>
double &xRef(lanelet_map_msgs::Node &point) {
    return point.point.x;
}

template<>
double &xRef(geometry_msgs::Pose &point) {
    return point.position.x;
}


template<>
const double &xRef(const lanelet_map_msgs::Node &point) {
    return point.point.x;
}

template<>
const double &xRef(const geometry_msgs::Pose &point) {
    return point.position.x;
}

template<>
double &yRef(lanelet_map_msgs::Node &point) {
    return point.point.y;
}

template<>
double &yRef(geometry_msgs::Pose &point) {
    return point.position.y;
}


template<>
const double &yRef(const lanelet_map_msgs::Node &point) {
    return point.point.y;
}

template<>
const double &yRef(const geometry_msgs::Pose &point) {
    return point.position.y;
}

}