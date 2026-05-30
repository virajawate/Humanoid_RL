# BSD 3-Clause License
# (License retained as per original)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'description_package',
            default_value='g1_description',
            description='Package with robot URDF/xacro files.',
        ),
        DeclareLaunchArgument(
            'description_file',
            default_value='g1.urdf',
            description='URDF/XACRO file with the robot description.',
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix to be added to joint names.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true.',
        ),
    ]

    # Launch configuration variables
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    prefix = LaunchConfiguration('prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Create robot_description param using xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare(description_package), 'urdf', description_file]),
        ' ',
        'prefix:=', prefix
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'publish_frequency': 100.0
        }]
    )

    # joint_state_publisher_gui node (optional for testing)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # rviz2 node
    rviz_config = PathJoinSubstitution([
        FindPackageShare(description_package), 'rviz', 'default.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # static tf node1 and node 2 are required because the base_link of G1 Robot
    # is not in the normal orientation as how it is in the Quadruped Robots.
    # These static transforms ensure that the MOLA SLAM Localization works as expected.

    static_tf_node1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_dummy_base_link_tf',
        arguments=['0.0', '0.0', '0.0', '0.003', '0.0', '3.14', 'base_link', 'dummy_base_link'],
        output='screen'
    )

    static_tf_node2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='dummy_base_link_to_livox_frame_tf',
        arguments=['-0.004', '-0.001', '-0.460', '3.14', '-3.14', '3.14', 'dummy_base_link', 'livox_frame'],
        output='screen'
    )

    return LaunchDescription(declared_arguments + [
        robot_state_publisher_node,
        static_tf_node1,
        static_tf_node2
        # TimerAction(period=2.0, actions=[joint_state_publisher_gui_node]),
        # TimerAction(period=3.0, actions=[rviz_node])
    ])
