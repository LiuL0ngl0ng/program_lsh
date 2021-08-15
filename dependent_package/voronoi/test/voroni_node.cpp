//
// Created by yangt on 19-6-6.
//
#include <ros/ros.h>
#include <internal_grid_map/internal_grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include "voronoi/voronoi_diagram.hpp"

class VoronoiNode {
 public:
    explicit VoronoiNode(ros::NodeHandlePtr &nh) {
        this->nh_ = nh;
        map_update_ = false;
        map_sub_ =
            nh_->subscribe("/local_grid_map", 1, &VoronoiNode::gridMapCb, this);
        pcl_pub_ = nh->advertise<sensor_msgs::PointCloud2>("/potential", 1, true);
    }

    void gridMapCb(const grid_map_msgs::GridMap &map_msgs) {
        this->map_msgs_ = map_msgs;
        this->map_update_ = true;
    }

    void run();

 private:
    ros::Subscriber map_sub_;
    ros::Publisher pcl_pub_;
    grid_map_msgs::GridMap map_msgs_;
    ros::NodeHandlePtr nh_;
    bool map_update_;
};

void VoronoiNode::run() {
    ros::Rate rate(50);
    while (ros::ok()) {
        ros::spinOnce();
        if(this->map_update_) {
            grid_map::GridMap gridmap;
            grid_map::GridMapRosConverter::fromMessage(this->map_msgs_,
                                                       gridmap);
            hmpl::InternalGridMap in_gm;
            in_gm.init(gridmap.getFrameId(), gridmap.getLength(),
                              gridmap.getResolution());
            in_gm.maps.get(in_gm.obs) = gridmap.get(in_gm.obs);
            in_gm.maps.setPosition(gridmap.getPosition());
            in_gm.maps.setFrameId(gridmap.getFrameId());
            in_gm.updateDistanceLayer();

            // voronoi
            VoronoiDiagram voronoi(in_gm.maps, 3);
            if(voronoi.isValid()) {
                grid_map::GridMap potential;
                voronoi.getPotentialField(in_gm.maps, &potential);
                potential.get("potential") *= 10.0;
                sensor_msgs::PointCloud2 pcl_msg;
                grid_map::GridMapRosConverter::toPointCloud(potential,
                                                            "potential",
                                                            pcl_msg);
                pcl_pub_.publish(pcl_msg);
            } else {
                ROS_WARN("Voroni failed!");
            }
        }
        rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "voronoi_test_node");
    ros::NodeHandlePtr nh(new ros::NodeHandle(""));
    VoronoiNode voronoi_node(nh);
    voronoi_node.run();
    /// main loop.
    return 0;
}