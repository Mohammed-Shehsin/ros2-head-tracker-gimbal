from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():

    pkg_path = get_package_share_directory('gimbal_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'gimbal.urdf.xacro')

    # Process Xacro → URDF
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    params = {'robot_description': robot_description_raw}

    rviz_config = os.path.join(pkg_path, 'rviz', 'display.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
