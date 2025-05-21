import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Define the RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='odom_vis',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('dim_cpp'), 'rviz', 'view_odom.rviz')]
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(rviz_node)

    return ld