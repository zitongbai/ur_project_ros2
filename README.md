# ur_project_ros2
The application of ur robots, robotiq gripper, realsense camera, etc. 

To find more infomation (installation, usage), visit [wiki](https://github.com/zitongbai/ur_project_ros2/wiki)

# Structure


* `robot_setup`: robot description, moveit config, simulation env setup, etc. 
    * `ur_robotiq_rs`: ur manipulator with a robotiq gripper and a realsense camera in its end effector
    * `dual_ur_robotiq_rs`: dual ur_robotiq_rs
    * `vision`: vision tools to detect objects, estimate pose, etc. 
* `tasks`:
    * `task_1_pick_and_place` use moveit to plan a path to the given pose, grasp the obj.
* `libs`: third-party tools, plugins, etc. 
