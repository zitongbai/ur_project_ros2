# ur_project_ros2
The application of ur robots, robotiq gripper, realsense camera, etc. 

To find more infomation (installation, usage), visit [wiki](https://github.com/zitongbai/ur_project_ros2/wiki)

> ⚠️ **Notice:** This repository is no longer actively maintained.  
> While you are welcome to explore and use the code, please note that updates and issue responses may be infrequent.  
> Thank you for your interest and understanding.


# Structure


* `robot_setup`: robot description, moveit config, simulation env setup, etc. 
    * `ur_robotiq_rs`: ur manipulator with a robotiq gripper and a realsense camera in its end effector
    * `dual_ur_robotiq_rs`: dual ur_robotiq_rs
    * `vision`: vision tools to detect objects, estimate pose, etc. 
* `tasks`:
    * `pick_and_place` use moveit to plan a path to the given pose, grasp the obj.
    * `find_hole` use ft sensor.
    * `dual_plan` plan and execute dual arms at the same time.
* `libs`: third-party tools, plugins, etc. 

# Temp note

create new task package: 
```shell
ros2 pkg create ${task_name} --build-type ament_cmake --node-name ${task_name} --dependencies rclcpp moveit_core moveit_ros_planning_interface moveit_common moveit_ros_planning tf2 rclcpp_action rclcpp_components control_msgs ${robot_pkg}
```
