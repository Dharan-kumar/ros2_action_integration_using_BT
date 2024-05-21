cmake_minimum_required(VERSION 3.8)
project(action_integration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
#find_package(rclcpp_components REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

find_package(rosidl_default_generators REQUIRED)


install(DIRECTORY
behavior_tree_xml launch
  DESTINATION share/${PROJECT_NAME}
)

# Install C++ behaviors
set(BEHAVIOR_SOURCES
src/behavior_tree_node.cpp 
)

#for including header file
include_directories(include)

add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server rclcpp rclcpp_action ros2_msgs)

add_executable(action_client src/action_bt_integration_node.cpp ${BEHAVIOR_SOURCES})
ament_target_dependencies(action_client rclcpp rclcpp_action ros2_msgs behaviortree_cpp_v3)


#rosidl_target_interfaces(sub ${PROJECT_NAME} "rosidl_typesupport_cpp")


#target for all the nodes
install(TARGETS
  action_server
  action_client
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()