from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare(package='arm_description')

    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'arm.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'arm.rviz'])

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'source_list': ['/ik_joint_states']}]
        ),

        Node(
            package='arm_description',
            executable='ik_interactive_node.py',
            name='ik_interactive_node',
            output='screen'
        ),

        Node(
            package='arm_description',
            executable='serial_bridge_node.py',
            name='serial_bridge_node',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])