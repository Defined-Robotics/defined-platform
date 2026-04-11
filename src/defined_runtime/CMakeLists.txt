cmake_minimum_required(VERSION 3.8)
project(defined_runtime)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(explore_lite_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Shared library — contains all BT action nodes and the status publisher.
# Both the executor and all test targets link against this single target so
# that include directories and ament dependencies are declared only once.
# ---------------------------------------------------------------------------
add_library(defined_runtime_lib
  src/goto_action.cpp
  src/wait_action.cpp
  src/report_action.cpp
  src/explore_action.cpp
  src/task_status_publisher.cpp
)
target_include_directories(defined_runtime_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(defined_runtime_lib
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav2_msgs
  nav_msgs
  behaviortree_cpp
  explore_lite_msgs
)
# Production builds: TESTABLE_STATIC functions keep static (internal) linkage.
target_compile_definitions(defined_runtime_lib PRIVATE PRODUCTION_CODE=1)

# BT Runtime Executor node
add_executable(bt_executor
  src/bt_executor_node.cpp
)
target_link_libraries(bt_executor defined_runtime_lib)
target_compile_definitions(bt_executor PRIVATE PRODUCTION_CODE=1)

install(TARGETS bt_executor defined_runtime_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Test targets link against defined_runtime_lib (no need to repeat sources or
  # target_include_directories).  PRODUCTION_CODE=0 exposes TESTABLE_STATIC
  # symbols for white-box unit testing.
  ament_add_gtest(test_wait_action test/test_wait_action.cpp)
  target_link_libraries(test_wait_action defined_runtime_lib)
  target_compile_definitions(test_wait_action PRIVATE PRODUCTION_CODE=0)

  ament_add_gtest(test_report_action test/test_report_action.cpp)
  target_link_libraries(test_report_action defined_runtime_lib)
  target_compile_definitions(test_report_action PRIVATE PRODUCTION_CODE=0)

  ament_add_gtest(test_goto_action test/test_goto_action.cpp)
  target_link_libraries(test_goto_action defined_runtime_lib)
  target_compile_definitions(test_goto_action PRIVATE PRODUCTION_CODE=0)

  ament_add_gtest(test_task_status test/test_task_status_publisher.cpp)
  target_link_libraries(test_task_status defined_runtime_lib)
  target_compile_definitions(test_task_status PRIVATE PRODUCTION_CODE=0)

  ament_add_gtest(test_bt_executor_integration test/test_bt_executor_integration.cpp)
  target_link_libraries(test_bt_executor_integration defined_runtime_lib)
  target_compile_definitions(test_bt_executor_integration PRIVATE PRODUCTION_CODE=0)

  ament_add_gtest(test_explore_action test/test_explore_action.cpp)
  target_link_libraries(test_explore_action defined_runtime_lib)
  target_compile_definitions(test_explore_action PRIVATE PRODUCTION_CODE=0)

  install(DIRECTORY test/fixtures/ DESTINATION share/${PROJECT_NAME}/test/fixtures)
endif()

ament_package()
