import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')
    
    octomap_rviz_config_path = os.path.join(
        package_path, 'rviz', 'octomap.rviz')

    # 声明配置变量（先声明，后引用）
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')  # 引用的参数必须先声明
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    # 1. 声明 use_sim_time 参数（保留）
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # 2. 声明 config_path 参数（保留）
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )

    # 3. 修正拼写错误：decalre -> declare
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )

    # 4. 恢复并声明 rviz 参数（核心修复！）
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',  # 默认启动rviz，可通过启动命令覆盖
        description='Use RViz to monitor results'
    )

    # 5. 合并 rviz_cfg 声明（避免重复）
    # 可通过启动命令指定rviz_cfg，默认用fastlio.rviz，也可改为octomap_rviz_config_path
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', 
        default_value=octomap_rviz_config_path,  # 若需要octomap默认，改为octomap_rviz_config_path default_rviz_config_path
        description='RViz config file path'
    )

    # Fast-LIO 核心节点（保留）
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # RViz 节点（仅当rviz_use为true时启动）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    # 组装LaunchDescription
    ld = LaunchDescription()
    # 添加所有参数声明
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)  # 核心：添加rviz参数声明
    ld.add_action(declare_rviz_config_path_cmd)

    # 添加节点
    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)

    return ld
