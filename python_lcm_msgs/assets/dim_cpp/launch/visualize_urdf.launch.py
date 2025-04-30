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
    # Get the path to the xacro file
    xacro_file = os.path.join(
        get_package_share_directory('dim_cpp'),
        'urdf',
        'alfred_base_descr.urdf.xacro'
    )

    # Process the xacro file to generate the robot description
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Define the joint state publisher GUI node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Define the robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Define the RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('dim_cpp'), 'rviz', 'visualize.rviz')]
    )

    head_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('depthai_examples'), 'launch', 'stereo_inertial_node.launch.py'])),
        launch_arguments={
            'mxId': '1844301011A6331300',
            'tf_prefix': 'head_cam',
            'parent_frame': 'head_cam_link',
            'base_frame': 'head_cam_frame',
            'cam_pos_x': '0',
            'cam_pos_y': '-0.022921',
            'cam_pos_z': '0.004764',
            'cam_roll': '0',
            'cam_pitch': '0',
            'cam_yaw': '-1.5707963267948966',
            'enableRviz': 'false',
        }.items(),
    )

    chest_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('depthai_examples'), 'launch', 'stereo_inertial_node.launch.py'])),
        launch_arguments={
            'mxId': '18443010A1043B1300',
            'tf_prefix': 'chest_cam',
            'parent_frame': 'chest_cam_link',
            'base_frame': 'chest_cam_frame',
            'cam_pos_x': '0',
            'cam_pos_y': '-0.022921',
            'cam_pos_z': '0.004764',
            'cam_roll': '0',
            'cam_pitch': '0',
            'cam_yaw': '-1.5707963267948966',
            'enableRviz': 'false',
        }.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the nodes to the launch description
    # ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    # ld.add_action(head_cam_launch)
    # ld.add_action(chest_cam_launch)

    return ld