import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pkg_path = get_package_share_directory('gen3_description')
    
  
    xacro_file = os.path.join(pkg_path, 'description', 'gen3_with_gripper.urdf.xacro')
    
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    # =================================

    # Node for the robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description] # Pass the processed URDF to the node
    )

    # Node for the joint_state_publisher_gui for manual control
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )
    
    # Node for RViz
    rviz_config_file = os.path.join(pkg_path, 'config', 'display.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Node for cam2ee transform broadcaster

    # cam2ee_node = Node(
    #     package = "gen3_description",
    #     executable= "cam2ee_tf_pub",
    #     name = "cam2ee_tf_pub"
    # )

    return LaunchDescription([
        # joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        # cam2ee_node
    ])