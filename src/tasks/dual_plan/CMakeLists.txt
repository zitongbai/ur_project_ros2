cmake_minimum_required(VERSION 3.8)
project(dual_plan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(control_msgs REQUIRED)
find_package(dual_ur_robotiq_rs_description REQUIRED)

add_executable(dual_plan src/dual_plan.cpp)
target_include_directories(dual_plan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dual_plan PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  dual_plan
  "rclcpp"
  "moveit_core"
  "moveit_ros_planning_interface"
  "moveit_common"
  "moveit_ros_planning"
  "tf2"
  "rclcpp_action"
  "rclcpp_components"
  "control_msgs"
  "dual_ur_robotiq_rs_description"
)

install(TARGETS dual_plan
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY 
    launch
    config
    gazebo
    rviz
  DESTINATION 
    share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
