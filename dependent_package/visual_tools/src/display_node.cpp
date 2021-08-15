/* Copyright (c) 2018, Yu Zhang, Intelligent Vehicle Research Center(IVRC), 
 * Beijing Institute of Technology.
 * All rights reserved.
 *        Files: display_node.cpp
 *   Created on: 11, 22, 2018 
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#include <visual_tools/display_node.hpp>
#include <memory>

namespace hmpl {

DemoNode::DemoNode(const ros::NodeHandle &node_handle,
                   const ros::NodeHandle &private_node_handle) {
    visual_tool_ptr_ = std::make_shared<VisualTools>(node_handle, private_node_handle);
    this->init();
}
void DemoNode::init() {
    periodic_timer_ = pnh_.createTimer(ros::Duration(0.1),
                                       &DemoNode::periodicTimerCallback,
                                       this);
                                       
}

void DemoNode::periodicTimerCallback(const ros::TimerEvent &event) {
    (void)event;
    visual_tool_ptr_->publishDemoMarkers();
    visual_tool_ptr_->display();
}
}