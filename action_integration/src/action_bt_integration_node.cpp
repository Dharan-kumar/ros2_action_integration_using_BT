#include "action_bt_integration_node.hpp"

using namespace std::chrono_literals;

const std::string tree_dir =
    ament_index_cpp::get_package_share_directory("action_integration") + "/behaviour_tree_xml";

ActionClientNode::ActionClientNode(const std::string &nodeName) : Node(nodeName)
{
  RCLCPP_INFO(get_logger(), "Init done");
}

void ActionClientNode::setUp()
{
  RCLCPP_INFO(get_logger(), "Setting up");
  createBehaviorTree();
  RCLCPP_INFO(get_logger(), "BT created");

  const auto timer_period = 500ms;
  m_timer = this->create_wall_timer(timer_period, std::bind(&ActionClientNode::updateBehaviorTree, this));

  rclcpp::spin(shared_from_this());
  rclcpp::shutdown();
}

void ActionClientNode::createBehaviorTree()
{
  BT::BehaviorTreeFactory factory;

  // registering bt node

  BT::NodeBuilder builder =
    [=](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<BehaviorTreeAction>(name, config, shared_from_this());
  };

  factory.registerBuilder<BehaviorTreeAction>("BehaviorTreeAction", builder);

  std::string check_string = tree_dir + "/tree.xml";
  RCLCPP_INFO(get_logger(), check_string.c_str());

  RCLCPP_INFO(get_logger(), tree_dir.c_str());

  //m_tree = factory.createTreeFromFile(tree_dir + "/tree.xml");
  m_tree = factory.createTreeFromFile("/home/dharan/microgenesis_ws/src/action_integration/behavior_tree_xml/tree.xml");
  
}

void ActionClientNode::updateBehaviorTree()
{
  BT::NodeStatus tree_status = m_tree.tickRoot();

  if (tree_status == BT::NodeStatus::RUNNING)
  {
    return;
  }
  else if (tree_status == BT::NodeStatus::SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(), "Data Transfered");
  }
  else if (tree_status == BT::NodeStatus::FAILURE)
  {
    RCLCPP_INFO(this->get_logger(), "Data Transfering Failed");
    m_timer->cancel();
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ActionClientNode>("action_client");
  node->setUp();

  return 0;
}