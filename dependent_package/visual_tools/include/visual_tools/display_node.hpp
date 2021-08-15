/* Copyright (c) 2018, Yu Zhang, Intelligent Vehicle Research Center(IVRC), 
 * Beijing Institute of Technology.
 * All rights reserved.
 *        Files: display_node.hpp
 *   Created on: 11, 22, 2018 
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#ifndef VISUAL_TOOLS_DISPLAY_NODE_HPP
#define VISUAL_TOOLS_DISPLAY_NODE_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include <visual_tools/visual_tools.h>

namespace hmpl {
class DemoNode {
 public:
    DemoNode(const ros::NodeHandle &node_handle,
             const ros::NodeHandle &private_node_handle);
    ~DemoNode() = default;
    void init();
    void periodicTimerCallback(const ros::TimerEvent& event);
 
 private:
    // public ros node handle
    ros::NodeHandle nh_;
    // private ros node handle
    ros::NodeHandle pnh_;
    std::string node_name_{"Demo Node"};
    // timer
    ros::Timer periodic_timer_;
    // publisher
    ros::Publisher periodic_pub_;
    // visual tool
    std::shared_ptr<VisualTools> visual_tool_ptr_;
};  // class DemoNode
}  // namespace demo


#endif  // VISUAL_TOOLS_DISPLAY_NODE_HPP
