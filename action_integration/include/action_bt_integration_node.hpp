#ifndef ACTION_BT_INTEGRATION_NODE_H
#define ACTION_BT_INTEGRATION_NODE_H

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <rclcpp/rclcpp.hpp>
#include "ros2_msgs/action/behavior_action.hpp"
#include <rclcpp/executors.hpp>
#include "action_server.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behavior_tree_node.hpp"

class ActionClientNode : public rclcpp::Node
{
public:
    using MyAction = ros2_msgs::action::BehaviorAction;
    explicit ActionClientNode(const std::string& node_name);
    void setUp();
    void createBehaviorTree();
    void updateBehaviorTree();

private:
    rclcpp::TimerBase::SharedPtr m_timer;
    BT::Tree m_tree;
};

#endif