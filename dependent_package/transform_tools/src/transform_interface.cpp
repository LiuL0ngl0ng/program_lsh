//
// Created by yangt on 8/9/18.
//
#include "transform_tools/transform_interface.hpp"
namespace cartographer {
namespace transform {
TransformTools::TransformTools() :
        a_(6378137.0),
        e2_(0.0818192 * 0.0818192),
        hemi_(HEMI_NORTH),
        zone_(UTM_ZONE_AUTO) {

}
void TransformTools::setParams(double a, double e2, GridZone zone,
                               Hemisphere hemi) {
    this->a_ = a;
    this->e2_ = e2;
    this->zone_ = zone;
    this->hemi_ = hemi;
}
void TransformTools::geographic_to_utm(double lon, double lat,
                                       double *utm_x, double *utm_y) {
    double lon_rad = lon * M_PI / 180.0;
    double lat_rad = lat * M_PI / 180.0;
    geographic_to_grid(a_, e2_, lat_rad, lon_rad, &zone_, &hemi_, utm_y, utm_x);
}
void TransformTools::utm_to_geograpbic(double utm_x, double utm_y,
                                       GridZone zone, double *lon,
                                       double *lat) {
    double lon_rad, lat_rad;
    if(grid_to_geographic(a_,e2_,zone, hemi_, utm_y, utm_x,
                          &lat_rad, &lon_rad)) {
        *lat = lat_rad * 180.0 / M_PI;
        *lon = lon_rad * 180.0 / M_PI;
    }
}
Rigid3d TransformTools::setTransform(const geometry_msgs::Pose &pose) {
    Rigid3d tf;
    Eigen::Vector3d pt(pose.position.x,
                       pose.position.y,
                       pose.position.z);
    Eigen::Quaternion<double> quaternion(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z);
    tf = Rigid3d(pt, quaternion);
    return tf;
}
Rigid3d TransformTools::setTransform(const geometry_msgs::Point &point,
                                     double yaw) {
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    Rigid3d tf;
    Eigen::Vector3d pt(point.x, point.y, point.z);
    Eigen::Quaternion<double> quaternion(q.w(), q.x(), q.y(), q.z());
    tf = Rigid3d(pt, quaternion);
    return tf;

}
}
}
