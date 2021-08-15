/* Copyright (c) 2017, Yu Zhang, Intelligent Vehicle Research Center(IVRC),
 * Beijing Institute of Technology.
 * All rights reserved.
 *        Files: car_geometry.cpp
 *   Created on: 05, 12, 2017
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#include "car_model/car_geometry.hpp"

namespace hmpl {

CarGeometry::CarGeometry()
    : wheelbase_(2.4),
      track_(2.0), //1.5748
      foot_print_{},
      centers_{},
      bounding_circle_{} {
    // default size of the car
    hmpl::Vector2D<double> rear_right(-2.16, -1.4); //-1.09, -1.255
    hmpl::Vector2D<double> front_right(2.14 , -1.4);
    hmpl::Vector2D<double> front_left(2.14 , 1.4);
    hmpl::Vector2D<double> rear_left(-2.16 , 1.4);
    this->foot_print_ = {rear_right, front_right, front_left, rear_left};
    // automatically generate the collision checking circles
    this->buildCirclesFromFootprint();
}

void CarGeometry::setFootprint(const hmpl::Vector2D<double> &rear_right,
                               const hmpl::Vector2D<double> &front_right,
                               const hmpl::Vector2D<double> &front_left,
                               const hmpl::Vector2D<double> &rear_left) {
    this->foot_print_ = {rear_right, front_right, front_left, rear_left};
}

std::vector<hmpl::Vector2D<double>> CarGeometry::getCurrentFootPrint(
        const hmpl::State &current) {
    std::vector<hmpl::Vector2D<double>> footprint;
    for(const auto &pt : foot_print_) {
        hmpl::State state_local;
        state_local.x = pt.x;
        state_local.y = pt.y;
        auto state_global = hmpl::localToGlobal(current, state_local);
        hmpl::Vector2D<double> result(state_global.x, state_global.y);
        footprint.push_back(result);
    }
    return footprint;
}
void CarGeometry::setCenters(const std::vector<hmpl::Circle> &centers) {
    this->centers_ = centers;
}
void CarGeometry::buildCirclesFromFootprint() {
    this->centers_.clear();
    // build the circle of two ends first, then two circles cover the left
    // middle
    // part
    double width = this->foot_print_[3].y - this->foot_print_[0].y;
    double length = this->foot_print_[1].x - this->foot_print_[0].x;

    // calculate the bounding circle and radius information
    this->bounding_circle_.r = sqrt(width/2 * width/2 + length/2 * length/2);
    this->bounding_circle_.position.x =
            (this->foot_print_[0].x + this->foot_print_[2].x) / 2.0;
    this->bounding_circle_.position.y =
            (this->foot_print_[0].y + this->foot_print_[2].y) / 2.0;
    
    hmpl::Circle center;

    // rear right
    center.position.x = this->foot_print_[0].x + width / 4.0;
    center.position.y = this->foot_print_[0].y + width / 4.0;
    double small_radius = this->foot_print_[0].Distance(center.position);
    center.r = small_radius;
    this->centers_.push_back(center);

    // front right
    center.position.x = this->foot_print_[1].x - width / 4.0;
    center.position.y = this->foot_print_[1].y + width / 4.0;
    center.r = small_radius;
    this->centers_.push_back(center);

    // front left
    center.position.x = this->foot_print_[2].x - width / 4.0;
    center.position.y = this->foot_print_[2].y - width / 4.0;
    center.r = small_radius;
    this->centers_.push_back(center);

    // rear left
    center.position.x = this->foot_print_[3].x + width / 4.0;
    center.position.y = this->foot_print_[3].y - width / 4.0;
    center.r = small_radius;
    this->centers_.push_back(center);

    double length_left = length - width;

    double R = sqrt(width * width + pow(length_left / 2.0, 2.0)) / 2.0;

    center.position.x =
            this->foot_print_[0].x + width / 2.0 + length_left / 4.0;
    center.position.y = 0;
    center.r = R;
    this->centers_.push_back(center);

    center.position.x =
            this->foot_print_[0].x + width / 2.0 + 3.0 * length_left / 4.0;
    center.position.y = 0;
    center.r = R;
    this->centers_.push_back(center);
    // divide the footprint
}
std::vector<hmpl::Circle> CarGeometry::getCurrentCenters(
        const hmpl::State &current) const {
    std::vector<hmpl::Circle> ego_circles;
    hmpl::Circle circle_buf;
    for (auto &circle_itr : this->centers_) {
        State temp;
        temp.x = circle_itr.position.x;
        temp.y = circle_itr.position.y;
        State circle_state = localToGlobal(current, temp);
        circle_buf.position.x = circle_state.x;
        circle_buf.position.y = circle_state.y;
        circle_buf.r = circle_itr.r;
        ego_circles.push_back(circle_buf);
    }
    return ego_circles;
}

hmpl::Circle CarGeometry::getBoundingCircle(const hmpl::State &current) const {
    hmpl::Circle circle_buf;
    State temp;
    temp.x = this->bounding_circle_.position.x;
    temp.y = this->bounding_circle_.position.y;
    State circle_state = localToGlobal(current, temp);
    circle_buf.position.x = circle_state.x;
    circle_buf.position.y = circle_state.y;
    circle_buf.r = this->bounding_circle_.r;
    return circle_buf;
}
}  // namespace hmpl
