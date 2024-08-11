import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess 
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(cmd=['ros2', 'run', 'lanes_ros2', 'ipm', '/zed/zed_node/rgb/'], output='screen'),
        print("hi from launch")
    ])
