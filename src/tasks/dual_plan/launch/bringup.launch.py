from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():


    dual_ur_robotiq_rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('dual_ur_robotiq_rs_description'), 
            '/launch/dual_ur_robotiq_rs.launch.py'
        ]), 
        launch_arguments={
            'runtime_config_package': 'dual_plan',
            'gazebo_world_file': 'sim_env.world',
            'base_frame_file': 'base_frame.yaml',
            'rviz_config_file': 'dual_plan.rviz',
            'use_sim_time': 'true',
        }.items()
    )

    return LaunchDescription([
        dual_ur_robotiq_rs_launch
    ])