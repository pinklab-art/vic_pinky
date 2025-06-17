import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('vicpinky_controller'),
        'config',
        'my_robot.urdf.xacro'
    )
    
    controller_config = os.path.join(
        get_package_share_directory('vicpinky_controller'),
        'config',
        'zlac_controllers.yaml'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}, controller_config],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
    ])