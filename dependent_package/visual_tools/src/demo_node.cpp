/* Copyright (c) 2018, Yu Zhang, Intelligent Vehicle Research Center(IVRC), 
 * Beijing Institute of Technology.
 * All rights reserved.
 *        Files: demo_node.cpp
 *   Created on: 11, 22, 2018 
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#include <visual_tools/display_node.hpp>
#include <string>

int main(int argc, char** argv) {
    std::string node_name = "hmpl_visual_tools";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    hmpl::DemoNode node(nh, nh_private);
    ROS_INFO("Initialized an async multi-thread node.");
    ros::AsyncSpinner s(4);  // Use 4 threads
    ROS_INFO_STREAM("Main loop in thread:" << boost::this_thread::get_id());
    s.start();
    ros::waitForShutdown();
}