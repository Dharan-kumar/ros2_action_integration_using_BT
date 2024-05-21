#include "action_server.hpp"

ActionServer::ActionServer(const rclcpp::NodeOptions & options): Node("MyAction_server", options)
{
    using namespace std::placeholders;

    RCLCPP_INFO(this->get_logger(),"Action Server Is Invoked");

    m_action_server = rclcpp_action::create_server<MyAction>(this,"my_action_server",
        std::bind(&ActionServer::handle_goal, this, _1, _2),
        std::bind(&ActionServer::handle_cancel, this, _1),
        std::bind(&ActionServer::handle_accepted, this, _1)
    );
}

rclcpp_action::GoalResponse ActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MyAction::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with order: %d", goal->start_value);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionServer::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MyAction>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionServer::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MyAction>> goal_handle)
{
    //using std::placeholders::_1;
    std::thread{std::bind(&ActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void ActionServer::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MyAction>> goal_handle)
{
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MyAction::Result>();
    auto feedback = std::make_shared<MyAction::Feedback>();
    auto & sequence = feedback->sequence_data;

    if (goal->start_value > 0)
    {
        for (int i = 0; i <= goal->start_value; ++i)
        {
            result->final_data.push_back(i);

            sequence.push_back(i);
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publishing feedback");
        }
    }
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal completed");
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActionServer>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}

