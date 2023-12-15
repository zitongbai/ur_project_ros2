# ur_project_ros2
The application of ur robots, robotiq gripper, realsense camera, etc. 

To find more infomation (installation, usage), visit [wiki](https://github.com/zitongbai/ur_project_ros2/wiki)

# Structure


* `robot_setup`: robot description, moveit config, simulation env setup, etc. 
    * `ur_robotiq_rs`: ur manipulator with a robotiq gripper and a realsense camera in its end effector
    * `dual_ur_robotiq_rs`: dual ur_robotiq_rs
    * `vision`: vision tools to detect objects, estimate pose, etc. 
* `tasks`:
    * `task_1_rs_calibration`
    * `task_2_hand_eye_calibration`
    * `task_3_multiple_cameras`
    * `task_4_grasp`
    * `task_5_pick_and_place`
* `libs`: third-party tools, plugins, etc. 
