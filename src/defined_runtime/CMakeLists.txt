cmake_minimum_required(VERSION 3.8)
project(defined_runtime)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# BT Runtime Executor node
add_executable(bt_executor
  src/bt_executor_node.cpp
  src/goto_action.cpp
  src/wait_action.cpp
  src/report_action.cpp
  src/task_status_publisher.cpp
)
target_include_directories(bt_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bt_executor
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav2_msgs
  behaviortree_cpp
)

install(TARGETS bt_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_wait_action test/test_wait_action.cpp
    src/wait_action.cpp)
  target_include_directories(test_wait_action PUBLIC include)
  ament_target_dependencies(test_wait_action rclcpp behaviortree_cpp std_msgs)

  ament_add_gtest(test_report_action test/test_report_action.cpp
    src/report_action.cpp)
  target_include_directories(test_report_action PUBLIC include)
  ament_target_dependencies(test_report_action rclcpp behaviortree_cpp std_msgs)

  ament_add_gtest(test_goto_action test/test_goto_action.cpp
    src/goto_action.cpp)
  target_include_directories(test_goto_action PUBLIC include)
  ament_target_dependencies(test_goto_action rclcpp rclcpp_action behaviortree_cpp nav2_msgs geometry_msgs)

  ament_add_gtest(test_task_status test/test_task_status_publisher.cpp
    src/task_status_publisher.cpp)
  target_include_directories(test_task_status PUBLIC include)
  ament_target_dependencies(test_task_status rclcpp std_msgs)

  ament_add_gtest(test_bt_executor_integration test/test_bt_executor_integration.cpp
    src/wait_action.cpp src/report_action.cpp src/goto_action.cpp src/task_status_publisher.cpp)
  target_include_directories(test_bt_executor_integration PUBLIC include)
  ament_target_dependencies(test_bt_executor_integration
    rclcpp rclcpp_action behaviortree_cpp std_msgs geometry_msgs nav2_msgs)

  install(DIRECTORY test/fixtures/ DESTINATION share/${PROJECT_NAME}/test/fixtures)
endif()

ament_package()
