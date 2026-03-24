import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 获取与拼接默认路径
    sentry_navigation_dir = get_package_share_directory('sentry_navigation')
    slam_pkg_share = get_package_share_directory('sentry_slam')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    icp_relocalization_dir = get_package_share_directory('icp_relocalization')

    # 创建 Launch 配置
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(slam_pkg_share, 'maps', 'map_1773548932.yaml'))
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(sentry_navigation_dir, 'config', 'nav2_params.yaml'))

    # ICP relocalization PCD地图路径 - 需要根据实际情况修改
    pcd_map_path = LaunchConfiguration('pcd_map_path', default='/home/rm/sentry_all/sentry_nav_3.0.3/PCD')

    # 创建导航启动描述 (不包含AMCL，使用ICP进行定位)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_path
        }.items(),
    )

    # 注意：不再需要静态发布 odom->base_link，因为 Fast-LIO 会动态发布这个变换
    # 如果需要初始位置，可以通过 icp_node 的 initial_x, initial_y, initial_z, initial_a 参数设置
    
    # 之前的静态变换（已删除，因为 Fast-LIO 会发布 odom->base_link）:
    # static_transform_node = launch_ros.actions.Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='odom_to_base_link',
    #     arguments=['0.0', '0', '0', '0', '0', '0', 'odom', 'base_link']  # XYZ RPY
    # )
    
    # ICP Relocalization - transform_publisher 发布 map->odom 变换
    icp_transform_publisher = launch_ros.actions.Node(
        package='icp_relocalization',
        executable='transform_publisher',
        name='transform_publisher',
        output='screen'
    )

    # ICP 节点 - 进行点云匹配
    icp_node = launch_ros.actions.Node(
        package='icp_relocalization',
        executable='icp_node',
        name='icp_node',
        output='screen',
        parameters=[
            {'initial_x': 0.0},
            {'initial_y': 0.0},
            {'initial_z': 0.0},
            {'initial_a': 0.0},
            {'map_voxel_leaf_size': 0.5},
            {'cloud_voxel_leaf_size': 0.3},
            {'map_frame_id': 'map'},
            {'solver_max_iter': 75},
            {'max_correspondence_distance': 0.1},
            {'RANSAC_outlier_rejection_threshold': 1.0},
            {'map_path': pcd_map_path},
            {'fitness_score_thre': 0.1},  # 匹配得分阈值，越小越严格
            {'converged_count_thre': 50},
            {'pcl_type': 'livox'},
        ],
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
        cmd=['ros2', 'launch', 'fast_lio', 'mapping.launch.py', 'rviz:=true'],
        output='screen'
    )

    launch_octomap_server2 = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'octomap_server', 'octomap_mapping.launch.xml'],
        output='screen'
    )

    launch_autoaim = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'rm_vision_bringup', 'vision_bringup.launch.py'],
        output='screen'
    )

    # 创建一个动作组，用于按顺序启动节点和进程
    action_group = launch.actions.GroupAction([
        launch.actions.TimerAction(period=1.0, actions=[launch_livox]),
        launch.actions.TimerAction(period=2.0, actions=[launch_dm_imu]),
        launch.actions.TimerAction(period=4.0, actions=[launch_fast_lio2]),
        launch.actions.TimerAction(period=5.0, actions=[launch_octomap_server2]),
        launch.actions.TimerAction(period=5.0, actions=[launch_autoaim]),
        launch.actions.TimerAction(period=6.0, actions=[navigation_launch]),
    ])

    # 创建 RViz 节点
    rviz_node = launch_ros.actions.Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      arguments=['-d', rviz_config_dir],
      parameters=[{'use_sim_time': use_sim_time}],
      output='log'
     )

    # 声明 Launch 参数
    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('map', default_value=map_yaml_path, description='Full path to map file to load'),
        DeclareLaunchArgument('params_file', default_value=nav2_param_path, description='Full path to param file to load'),
        DeclareLaunchArgument('pcd_map_path', default_value=pcd_map_path, description='Full path to PCD map file for ICP relocalization'),
    ]

    # 返回 LaunchDescription - 方案二：Fast-LIO + ICP
    # 数据流：Livox -> Fast-LIO -> odom->base_link
    #         Livox -> ICP -> map->odom (校正)
    return launch.LaunchDescription([
        *declared_arguments,
        icp_transform_publisher,
        icp_node,
        rviz_node,
        action_group,
    ])
