from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'controllers_file',
            default_value='controllers.yaml',
            description='YAML file with the controllers configuration.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='dual_ur_robotiq_rs_description',
            description='Description package with robot URDF/xacro files. Usually the argument \
                         is not set, it enables use of a custom description.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='dual_ur_robotiq_rs.urdf.xacro',
            description='URDF/XACRO description file with the robot.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gazebo_world_file',
            default_value='empty.world',
            description='gazebo world file with the robot',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint names, useful for multi-robot setup. \
                         If changed than also joint names in the controllers \
                         configuration have to be updated. Expected format "<prefix>/"',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace of launched nodes, useful for multi-robot setup. \
                         If changed than also the namespace in the controllers \
                         configuration needs to be updated. Expected format "<ns>/".',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'sim_gazebo',
            default_value='true',
            description='Start robot in Gazebo simulation.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Start robot with fake hardware mirroring command to its states.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_planning',
            default_value='true',
            description='Start robot with Moveit2 `move_group` planning \
                         config for Pilz and OMPL.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'left_arm_controller',
            default_value='left_ur_arm_controller',
            description='arm controller to start.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'left_gripper_controller',
            default_value='left_gripper_controller',
            description='gripper controller to start.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'right_arm_controller',
            default_value='right_ur_arm_controller',
            description='arm controller to start.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'right_gripper_controller',
            default_value='right_gripper_controller',
            description='gripper controller to start.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        )
    )

    # Initialize Arguments
    controllers_file = LaunchConfiguration('controllers_file')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    gazebo_world_file = LaunchConfiguration('gazebo_world_file')
    prefix = LaunchConfiguration('prefix')
    namespace = LaunchConfiguration('namespace')
    sim_gazebo = LaunchConfiguration('sim_gazebo')
    use_planning = LaunchConfiguration('use_planning')
    left_arm_controller = LaunchConfiguration('left_arm_controller')
    left_gripper_controller = LaunchConfiguration('left_gripper_controller')
    right_arm_controller = LaunchConfiguration('right_arm_controller')
    right_gripper_controller = LaunchConfiguration('right_gripper_controller')
    start_rviz = LaunchConfiguration('start_rviz')

    # File path
    robot_controllers_file = PathJoinSubstitution(
        [FindPackageShare(description_package), 'config', controllers_file,]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), 'rviz', 'dual_ur_robotiq_rs.rviz']
    )
    gazebo_world_file = PathJoinSubstitution(
        [FindPackageShare(description_package), 'gazebo', gazebo_world_file]
    )

    # Get URDF/XACRO file path
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(description_package), 'urdf', description_file]
            ), 
            ' ',
            'ur_type:=',
            'ur5e',
            ' ',
            'sim_gazebo:=', 
            sim_gazebo, 
            ' ', 
            'simulation_controllers:=',
            robot_controllers_file,
            ' ',
        ]
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
        # it is the safe way to wrap the xacro output
        # ref: https://answers.ros.org/question/417369/caught-exception-in-launch-see-debug-for-traceback-unable-to-parse-the-value-of-parameter-robot_description-as-yaml/
    }
    
    # Nodes and Launch
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers_file],
        output='both',
        namespace=namespace,
        condition=UnlessCondition(sim_gazebo),
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[
            robot_description,
            {"use_sim_time": sim_gazebo},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            {"use_sim_time": sim_gazebo},
        ],
        condition=UnlessCondition(use_planning), # Do not start RViz2 if planning is used, 
                                                # because rviz is launched inside planning launch file
    )

    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare(description_package),
            '/launch',
            '/dual_ur_robotiq_rs_planning.launch.py'
        ]),
        launch_arguments={
            'description_package':description_package,
            'description_file': description_file,
            'prefix': prefix,
            'namespace': namespace,
            'use_sim': sim_gazebo,
            'start_rviz': start_rviz,
        }.items(),
        condition=IfCondition(use_planning),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution(
                [FindPackageShare('gazebo_ros'),
                    'launch', 'gazebo.launch.py']
            )]
        ),
        launch_arguments={'verbose': 'true', 'world': gazebo_world_file}.items(),
        condition=IfCondition(sim_gazebo),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name="spawn_ur_robotiq_rs",
        arguments=['-topic', [namespace, 'robot_description'], '-entity', [namespace, 'dual_ur_robotiq_rs']],
        output='screen',
        condition=IfCondition(sim_gazebo),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager',
                   [namespace, 'controller_manager']],
    )

    left_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[left_arm_controller, '--controller-manager', [namespace, 'controller_manager']],
    )

    left_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[left_gripper_controller, '--controller-manager', [namespace, 'controller_manager']],
    )

    right_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[right_arm_controller, '--controller-manager', [namespace, 'controller_manager']],
    )

    right_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[right_gripper_controller, '--controller-manager', [namespace, 'controller_manager']],
    )

    # Delay `joint_state_broadcaster` after spawn_entity if in simulation
    delay_joint_state_broadcaster_spawner_after_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        ),
        condition=IfCondition(sim_gazebo),
    )

    # Delay `joint_state_broadcaster` after control_node if not in simulation
    delay_joint_state_broadcaster_spawner_after_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[joint_state_broadcaster_spawner],
        ),
        condition=UnlessCondition(sim_gazebo),
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(start_rviz),
    )

    # Delay start of robot controllers after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[left_arm_controller_spawner, left_gripper_controller_spawner, right_arm_controller_spawner, right_gripper_controller_spawner],
        )
    )

    nodes = [
        gazebo,
        control_node,
        planning_launch,
        spawn_entity,
        robot_state_pub_node,
        delay_joint_state_broadcaster_spawner_after_control_node,
        delay_joint_state_broadcaster_spawner_after_spawn_entity,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)