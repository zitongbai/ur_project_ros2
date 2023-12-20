import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    # kinematics config, needed for move group in pick_and_place
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("dual_ur_robotiq_rs_description"), "moveit2", "kinematics.yaml"]
    )

    return LaunchDescription([
        Node(
            package='dual_plan',
            executable='dual_plan',
            name='dual_plan_node',
            parameters=[{
                    "use_sim_time":True,
                },
                robot_description_kinematics, 
            ], 
            output='screen'
        ),
    ])