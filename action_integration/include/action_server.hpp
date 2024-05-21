#ifndef ACTION_SERVER_H
#define ACTION_SERVER_H

#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
//#include "rclcpp_components/register_node_macro.hpp"
#include "ros2_msgs/action/behavior_action.hpp"

class ActionServer: public rclcpp::Node
{
public:
    using MyAction = ros2_msgs::action::BehaviorAction;
    using MyGoalHandler = rclcpp_action::ServerGoalHandle<MyAction>;

    ActionServer(const rclcpp::NodeOptions & options);

private:
    rclcpp_action::Server<MyAction>::SharedPtr m_action_server;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MyAction::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<MyAction>> goal_handle);
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MyAction>> goal_handle);
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MyAction>> goal_handle);
};

#endif