#ifndef BEHAVIOR_TREE_NODE_H
#define BEHAVIOR_TREE_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_msgs/action/behavior_action.hpp"
#include <string>

class BehaviorTreeAction : public BT::StatefulActionNode
{
public:
  BehaviorTreeAction(const std::string &name,
           const BT::NodeConfiguration &config,
           rclcpp::Node::SharedPtr node_ptr);

  using BehaviorAction = ros2_msgs::action::BehaviorAction;
  using GoalHandleBehaviorAction = rclcpp_action::ClientGoalHandle<BehaviorAction>;

  rclcpp::Node::SharedPtr m_node_ptr;
  rclcpp_action::Client<BehaviorAction>::SharedPtr m_action_client_ptr;
  bool done_flag_;

  // Method overrides
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override{};

  void goalResponseCallback(const GoalHandleBehaviorAction::SharedPtr & goal_handle);
  void feedbackCallback(GoalHandleBehaviorAction::SharedPtr, const std::shared_ptr<const BehaviorAction::Feedback> feedback);
  // Action Client callback
  void actionResultCallback(const GoalHandleBehaviorAction::WrappedResult& result);
};

#endif