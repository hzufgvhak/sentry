from launch import LaunchDescription
from launch_ros.actions import Node

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                # 将输入话题从默认的 /cloud 改为你的雷达话题
                ('cloud_in', '/cloud_registered'),
                # 输出话题默认为 /scan
                ('scan', '/scan'),
            ],
            parameters=[{
                'target_frame': 'base_link', # 投影到的参考系，通常选底盘
                'transform_tolerance': 0.01,
                'min_height': 0.0,            # 切片的最小高度（相对于 target_frame）
                'max_height': 0.6,            # 切片的最大高度（在这个范围内的点会被打平）
                'angle_min': -3.1415,         # -180度
                'angle_max': 3.1415,          # 180度
                'angle_increment': 0.0087,    # 0.5度一个点
                'scan_time': 0.1,
                'range_min': 0.2,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
        )
    ])