cmake_minimum_required(VERSION 3.8)
project(pick_and_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(ur_robotiq_rs_description REQUIRED)

# include_directories(
#   include
#   ${ur_robotiq_rs_description_INCLUDE_DIRS}
# )

# add_executable(pick_and_place src/pick_and_place.cpp)

# ament_target_dependencies(pick_and_place
#   rclcpp moveit_ros_planning_interface tf2 
#   moveit_core moveit_ros_planning control_msgs
#   ur_robotiq_rs_description
# )

# install(TARGETS pick_and_place
#   DESTINATION lib/${PROJECT_NAME}
# )

add_executable(pick_and_place src/pick_and_place.cpp)
target_include_directories(pick_and_place PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pick_and_place
  rclcpp tf2 
  moveit_core moveit_ros_planning control_msgs
  ur_robotiq_rs_description
)

install(TARGETS pick_and_place
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
