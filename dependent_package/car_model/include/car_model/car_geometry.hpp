/* Copyright (c) 2017, Yu Zhang, Intelligent Vehicle Research Center(IVRC),
 * Beijing Institute of Technology.
 * All rights reserved.
 *        Files: car_footprint.hpp
 *   Created on: 05, 09, 2017
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#ifndef CAR_MODEL_CAR_GEOMETRY_HPP
#define CAR_MODEL_CAR_GEOMETRY_HPP

#include <array>
#include <opt_utils/circle.hpp>
#include <opt_utils/opt_utils.hpp>
namespace hmpl {
/// This data type describe the footprint of the car in ego frame
///      y^
///       |
///    |---------------------| --
///    |  |                  | ^
///    |--+--> x         *   | track
///    |  |                  | v
///    |---------------------| --
///       |<- wheelbase->|
///    + : rear axle center
///    storage order of the vetex : rear_right  front_right  fornt_left
///    rear_left
///    in footprint array

class CarGeometry {
 public:
    CarGeometry();
    std::array<hmpl::Vector2D<double>, 4> getFootPrint() const {
        return this->foot_print_;
    };
    std::vector<hmpl::Circle> getCenters() const { return this->centers_; };

    /**
     * Get the center circle points in ego frame based on current state
     * @param ref The current vehicle state in ego frame
     * @return The centers in ego frame
     */
    std::vector<hmpl::Circle> getCurrentCenters(const hmpl::State &current) const;
    /// get the wheelbase info of the car
    double getWheelbase() const { return this->wheelbase_; };
    /// get the track info of the car
    double getTrack() const { return this->track_; };
    /// get the bounding circle for collision checking in ego frame
    hmpl::Circle getBoundingCircle() const { return this->bounding_circle_;};
   
    /**
     * Get the bounding circle position in global frame with respect to
     * current state.
     * @param current Current vehicle state in global frame
     * @return circlce position represented in global frame
     */
    hmpl::Circle getBoundingCircle(const hmpl::State &current) const;

    /**
     * Set the wheelbase parameter for a car
     * @param wheelbase Wheelbase info in meter
     */
    void setWheebase(double wheelbase) { this->wheelbase_ = wheelbase; };
    /**
     * Set the track parameter for a car
     * @param track track info in meter
     */
    void setTrack(double track) { this->track_ = track; }

    // set the Geometry
    /**
     * Set the foot print polygon of the car.  Start from rear_right vertex in
     * counter-clockwise: rear_right, front_left, front_left, rear_left
     * The coordinates are represented in local frame
     * @param rear_right vertex
     * @param front_right vertex
     * @param front_left vertex
     * @param rear_left vertex
     */
    void setFootprint(const hmpl::Vector2D<double> &rear_right,
                      const hmpl::Vector2D<double> &front_right,
                      const hmpl::Vector2D<double> &front_left,
                      const hmpl::Vector2D<double> &rear_left);

    // set the centers of the car
    /**
     * Provide a algorithm to build collision checking circle
     */
    void buildCirclesFromFootprint();

    /**
     * Set the collision checking circle center points represented in local
     * frame
     * @param centers The circles that cover the car footprint
     */
    void setCenters(const std::vector<hmpl::Circle> &centers);

    std::vector<hmpl::Vector2D<double>> getCurrentFootPrint(
            const hmpl::State &current);

 private:
    /// wheelbase of the car in meter
    double wheelbase_;
    /// tracks of the car in meter
    double track_;
    /// the foot print of the car, 4 vertices counter clock-wise
    std::array<hmpl::Vector2D<double>, 4> foot_print_;
    /// the collision checking circle centers, the order doesn't matter.
    std::vector<hmpl::Circle> centers_;
    /// the bounding circle
    hmpl::Circle bounding_circle_;
};

}  // namespace hmpl
#endif  // STATE_SAMPLING_CAR_GEOMETRY_HPP
