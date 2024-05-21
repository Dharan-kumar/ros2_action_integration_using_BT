#include "behavior_tree_node.hpp"

BehaviorTreeAction::BehaviorTreeAction(const std::string &name, const BT::NodeConfiguration &config, 
  rclcpp::Node::SharedPtr node_ptr): BT::StatefulActionNode(name, config), m_node_ptr(node_ptr)
{
  m_action_client_ptr = rclcpp_action::create_client<BehaviorAction>(m_node_ptr, "/my_action_server");
  done_flag_ = false;
}

BT::NodeStatus BehaviorTreeAction::onStart()
{
  //using namespace std::placeholders;
  auto send_goal_options = rclcpp_action::Client<BehaviorAction>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&BehaviorTreeAction::actionResultCallback, this, std::placeholders::_1);
  send_goal_options.goal_response_callback = std::bind(&BehaviorTreeAction::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(&BehaviorTreeAction::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);

  auto goal_msg = BehaviorAction::Goal();
  goal_msg.start_value =10;

  // send pose
  done_flag_ = false;

  m_action_client_ptr->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(m_node_ptr->get_logger(), "Input Goal is Sent\n");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BehaviorTreeAction::onRunning()
{
  if (done_flag_)
  {
    RCLCPP_INFO(m_node_ptr->get_logger(), "Sequence Data Transfered\n");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_INFO(m_node_ptr->get_logger(), "Sequence Data Running\n");
    return BT::NodeStatus::RUNNING;
  }
}
void BehaviorTreeAction::goalResponseCallback(const GoalHandleBehaviorAction::SharedPtr & goal_handle)
{
  if (!goal_handle) 
  {
    RCLCPP_ERROR(m_node_ptr->get_logger(), "Goal was rejected by server");
  } 
  else {
    RCLCPP_INFO(m_node_ptr->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void BehaviorTreeAction::feedbackCallback(GoalHandleBehaviorAction::SharedPtr, const std::shared_ptr<const BehaviorAction::Feedback> feedback)
{
  std::stringstream ss;
  ss << "Next number in sequence received: ";
  for (auto number : feedback->sequence_data)
  {
    ss << number << " ";
  }
  RCLCPP_INFO(m_node_ptr->get_logger(), ss.str().c_str());
}

void BehaviorTreeAction::actionResultCallback(const GoalHandleBehaviorAction::WrappedResult & result)
{ 
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(m_node_ptr->get_logger(),"Goal Was Succesfully Executed");
      done_flag_ = true;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(m_node_ptr->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(m_node_ptr->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(m_node_ptr->get_logger(), "Unknown result code");
      return;
  }
  std::stringstream ss;
  ss << "Result received: ";
  for (auto number : result.result->final_data)
  {
    ss << number << " ";
  }
  RCLCPP_INFO(m_node_ptr->get_logger(), ss.str().c_str());
}