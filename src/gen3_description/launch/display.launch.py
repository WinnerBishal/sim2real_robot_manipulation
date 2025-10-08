import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    # Find urdf
    pkg_path = get_package_share_directory('gen3_description')
    urdf_file = os.path.join(pkg_path, 'description', 'kinova_gen3.urdf')

    # Read urdf contents
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    # Create robot_state_publisher node (different from rclpy Node)

    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable='robot_state_publisher',
        name = 'robot_state_publisher',
        parameters = [{'robot_description': robot_desc}])
    
    # joint_state_publisher_gui node
    joint_state_publisher_gui_node = Node(
        package = 'joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # rviz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name = 'rviz2',
        arguments = ['-d', os.path.join(pkg_path, 'config', 'display.rviz')]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        # joint_state_publisher_gui_node,
        rviz_node
    ])