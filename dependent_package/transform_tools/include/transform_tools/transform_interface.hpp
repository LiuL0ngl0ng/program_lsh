//
// Created by yangt on 8/9/18.
//

#ifndef TRANSFORM_TOOLS_TRANSFORM_INTERFACE_HPP
#define TRANSFORM_TOOLS_TRANSFORM_INTERFACE_HPP
#include "transform_tools/transform.h"
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
namespace cartographer {
namespace transform {
class TransformTools {
 public:
    TransformTools();
    void setParams(double a, double e2, GridZone zone, Hemisphere hemi);
    void geographic_to_utm(double lon, double lat,
                           double *utm_x, double *utm_y);
    void utm_to_geograpbic(double utm_x, double utm_y, GridZone zone,
                           double *lon, double *lat);
    Rigid3d setTransform(const geometry_msgs::Pose &pose);
    Rigid3d setTransform(const geometry_msgs::Point &point, double yaw);
 private:
    double a_;
    double e2_;
    GridZone zone_;
    Hemisphere hemi_;
};
}
}
#endif //TRANSFORM_TOOLS_TRANSFORM_INTERFACE_HPP
