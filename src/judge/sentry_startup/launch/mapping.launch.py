# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
# from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
# from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration 

# def generate_launch_description():

#   config_path = os.path.join(
#       get_package_share_directory('sentry_startup'), 'config') 
  
#   # twist2chassis_cmd_node=Node(
#   #   package='cmd_chassis',
#   #   executable='twist2chassis_cmd',
#   #   output='screen'
#   # )
  
#   # fake_joint_node=Node(
#   #   package='cmd_chassis',
#   #   executable='fake_joint',
#   #   output='screen'
#   # )
  
#   # twist_transformer_node=Node(
#   #   package='cmd_chassis',
#   #   executable='twist_transformer',
#   #   output='screen'
#   # )

#   # rot_imu=Node(
#   #   package='cmd_chassis',
#   #   executable='rot_imu',
#   #   output='screen'
#   # )

#   # sentry_description = IncludeLaunchDescription(
#   #   PythonLaunchDescriptionSource([os.path.join(
#   #       get_package_share_directory('sentry_description'), 'launch', 'view_model.launch.py')])
#   #  )

#   # mid360
#   mid360_node = IncludeLaunchDescription(
#       PythonLaunchDescriptionSource([os.path.join(
#           get_package_share_directory('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360_launch.py')])
#   )

#   start_extra_imu = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(
#                 get_package_share_directory('dm_imu'),
#                 'launch',
#                 'run_without_rviz.launch.py'  # 注意：这是 .py 文件，应该用 PythonLaunchDescriptionSource
#             )
#         )
#     )
  
#   # fast-lio localization   
#   fast_lio_param = os.path.join(
#       config_path, 'fast_lio_mapping_param.yaml')
#   fast_lio_node = Node(
#         package='fast_lio',
#         executable='fastlio_mapping',
#         parameters=[
#           fast_lio_param
#         ],
#         output='screen',
#         #remappings=[('/Odometry','/state_estimation')]
#     )

#   # start_octomap_server = IncludeLaunchDescription(
#   #   PythonLaunchDescriptionSource([os.path.join(
#   #       get_package_share_directory('sentry_bringup'), 'launch', 'octomap_server_intensity.launch.py')])
#   # )

#   start_octomap_server = IncludeLaunchDescription(
#     # 注意这里改成了 FrontendLaunchDescriptionSource
#     FrontendLaunchDescriptionSource([os.path.join(
#         get_package_share_directory('octomap_server'), 'launch', 'octomap_mapping.launch.xml')])
# )

#   # launch_octomap_server2 = launch.actions.ExecuteProcess(
#   #       cmd=['ros2', 'launch', 'octomap_server', 'octomap_mapping.launch.xml'],
#   #       output='screen'
#   #   )


#   rviz_config_file = os.path.join(
#     get_package_share_directory('sentry_startup'), 'rviz', 'loam_livox.rviz')

#   start_rviz = Node(
#     package='rviz2',
#     executable='rviz2',
#     arguments=['-d', rviz_config_file,'--ros-args', '--log-level', 'warn'],
#     output='screen'
#   )

#   delayed_start_mapping = TimerAction(
#     period=8.0,
#     actions=[
#       fast_lio_node,
#       start_octomap_server
#     ]
#   )

#   ld = LaunchDescription()

#   #ld.add_action(twist2chassis_cmd_node)
#   #ld.add_action(fake_joint_node)
#   #ld.add_action(twist_transformer_node)
#   ld.add_action(start_extra_imu)
#   #ld.add_action(rot_imu)
#   #ld.add_action(sentry_description)
#   ld.add_action(mid360_node)
#   ld.add_action(start_rviz)
#   ld.add_action(delayed_start_mapping)

#   return ld

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # 获取与拼接默认路径
    sentry_navigation_dir = get_package_share_directory('sentry_navigation')
    slam_pkg_share = get_package_share_directory('sentry_slam')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    fast_lio_rviz = get_package_share_directory('fast_lio')
    rviz_cfg = os.path.join(fast_lio_rviz, 'rviz_cfg', 'loam_livox.rviz')
    #rviz_cfg = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    rviz_use = LaunchConfiguration('rviz_use', default='true')
    
    # 创建 Launch 配置
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    #map_yaml_path = LaunchConfiguration('map', default=os.path.join(slam_pkg_share, 'maps', 'map_20260131_151433.yaml'))
    #map_yaml_path = LaunchConfiguration('map', default=os.path.join(slam_pkg_share, 'maps', 'KONG.yaml'))
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(sentry_navigation_dir, 'config', 'nav2_params.yaml'))

    # 创建导航启动描述
    # navigation_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
    #     launch_arguments={
    #         'map': map_yaml_path,
    #         'use_sim_time': use_sim_time,
    #         'params_file': nav2_param_path
    #     }.items(),
    # )

    # 创建 SLAM 启动描述
  #   slam_launch = IncludeLaunchDescription(
  #       PythonLaunchDescriptionSource(
  #          os.path.join(slam_pkg_share, 'launch', 'cartographer.launch.py')
  #       )
  #  )

     # 创建静态变换节点
    static_transform_node = launch_ros.actions.Node(
       package='tf2_ros',
       executable='static_transform_publisher',
       name='odom_to_base_link',
       arguments=['0.0', '0', '0', '0', '0', '0', 'odom', 'base_link']  # XYZ RPY
     )
    

    # 启动 LiDAR、IMU、Fast LiO 和 Octomap Server 进程
    launch_livox = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'livox_ros_driver2', 'msg_MID360_launch.py'],
        output='screen'
    )

    launch_dm_imu = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'dm_imu', 'run_without_rviz.launch.py'],
        output='screen'
    )

    launch_fast_lio2 = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'fast_lio', 'mapping.launch.py'],
        output='screen'
    )

    launch_octomap_server2 = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'octomap_server', 'octomap_mapping.launch.xml'],
        output='screen'
    )

    # launch_autoaim = launch.actions.ExecuteProcess(
    #     cmd=['ros2', 'launch', 'rm_vision_bringup', 'vision_bringup.launch.py'],
    #     output='screen'
    # )

    # launch_judge = launch_ros.actions.Node(
    #     package='sentry_api',
    #     executable='judge',
    #     name='judge',
        
    #     output='log',
        # 如果需要参数可以在这里添加
        #parameters=[{'use_sim_time': use_sim_time}]
    #)
    # 创建一个动作组，用于按顺序启动节点和进程
    action_group = launch.actions.GroupAction([
        launch.actions.TimerAction(period=1.0, actions=[launch_livox]),
        launch.actions.TimerAction(period=2.0, actions=[launch_dm_imu]),
        launch.actions.TimerAction(period=4.0, actions=[launch_fast_lio2]),
        launch.actions.TimerAction(period=5.0, actions=[launch_octomap_server2]),
       # launch.actions.TimerAction(period=5.0, actions=[launch_autoaim]),
       # launch.actions.TimerAction(period=6.0, actions=[navigation_launch]),
        #launch.actions.TimerAction(period=7.0, actions=[slam_launch]),
        #launch.actions.TimerAction(period=18.0, actions=[launch_judge]),
    ])

    # 创建 RViz 节点
    # rviz_node = launch_ros.actions.Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_dir],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='log'
    # )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    # 声明 Launch 参数
    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='Use simulation (Gazebo) clock if true'),
        #DeclareLaunchArgument('map', default_value=map_yaml_path, description='Full path to map file to load'),
      #  DeclareLaunchArgument('params_file', default_value=nav2_param_path, description='Full path to param file to load'),
    ]

    # 返回 LaunchDescription
    return launch.LaunchDescription([
        *declared_arguments,
        static_transform_node,
        #rviz_node,
        action_group,
        #launch_judge
    ])