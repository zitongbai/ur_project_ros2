from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():


    ur_ft_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ur_ft_description'), 
            '/launch/ur_ft.launch.py'
        ]), 
        launch_arguments={
            'runtime_config_package': 'find_hole',
            'gazebo_world_file': 'sim_env.world',
            'base_frame_file': 'base_frame.yaml',
            'rviz_config_file': 'find_hole.rviz',
            'use_sim_time': 'true',
        }.items()
    )



    return LaunchDescription([
        ur_ft_launch
    ])