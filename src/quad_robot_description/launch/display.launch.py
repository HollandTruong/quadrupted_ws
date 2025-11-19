from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

def generate_launch_description():
    share_dir = get_package_share_directory('quad_robot_description')

    # URDF/XACRO và RViz config
    xacro_file = os.path.join(share_dir, 'urdf', 'quad_robot.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    # GUI argument
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )
    show_gui = LaunchConfiguration('gui')

    # robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file])
        }]
    )

    # joint_state_publisher nodes
    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # quad_odom node
    quad_odom_node = Node(
        package='quad_odom',
        executable='odom_node',
        name='quad_odom',
        output='screen'
    )

    # rviz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        quad_odom_node,
        rviz_node
    ])

