import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    package_dir = get_package_share_directory("akari_simulator")
    urdf = os.path.join(package_dir, "urdf", "akari.urdf")
    rviz = os.path.join(package_dir, "rviz", "akari.rviz")
    

    condition = LaunchConfiguration('use_gui')
    condition_argument = DeclareLaunchArgument("use_gui", default_value="true")
    
    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf]),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            arguments=[urdf],
            condition=IfCondition(condition)), # use_gui:=true
        
        Node(
            package='akari_simulator',
            executable='joint_state_publisher_gui_subscriber',
            name='joint_state_publisher_gui_subscriber',
            output='screen',
            condition=IfCondition(condition)),  # use_gui:=true
            
        Node(
            package='akari_simulator',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            arguments=[urdf],
            condition=UnlessCondition(condition)), # use_gui:=false
        
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=['-d' + rviz]),
    ])
