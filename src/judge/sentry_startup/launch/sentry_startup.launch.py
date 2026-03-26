import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    sentry_navigation_dir = get_package_share_directory('sentry_navigation')
    slam_pkg_share = get_package_share_directory('sentry_slam')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # 鍒涘缓 Launch 閰嶇疆
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(slam_pkg_share, 'maps', 'rmul_2026.yaml'))
    #map_yaml_path = LaunchConfiguration('map', default=os.path.join(slam_pkg_share, 'maps', 'KONG.yaml'))
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(sentry_navigation_dir, 'config', 'nav2_params.yaml'))

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_path
        }.items(),
    )

   
    # slam_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(slam_pkg_share, 'launch', 'cartographer.launch.py')
    #     )
    # )

   
    static_transform_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link',
        arguments=['0.0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
    

   
    launch_livox = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'livox_ros_driver2', 'msg_MID360_launch.py'],
        output='screen'
    )

    # launch_scan = launch.actions.ExecuteProcess(
    #     cmd=['ros2', 'launch', 'sentry_startup', 'pointcloud_to_laserscan.launch.py'],
    #     output='screen'
    #)
    

    launch_dm_imu = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'dm_imu', 'run_without_rviz.launch.py'],
        output='screen'
    )

    # rot_imu
    rot_imu_node = launch_ros.actions.Node(
        package='cmd_chassis',
        executable='rot_imu',
        name='rot_imu',
        output='screen'
    )

    launch_fast_lio2 = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'fast_lio', 'mapping.launch.py','rviz:=true'],#淇敼浜唕viz
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


    launch_judge = launch_ros.actions.Node(
        package='sentry_api',
        executable='judge',
        name='judge',
        output='log',
      
        #parameters=[{'use_sim_time': use_sim_time}]
    )
    action_group = launch.actions.GroupAction([
        launch.actions.TimerAction(period=1.0, actions=[launch_livox]),
        launch.actions.TimerAction(period=2.0, actions=[launch_dm_imu]),
        # rot_imu     
        launch.actions.TimerAction(period=3.0, actions=[rot_imu_node]),
        launch.actions.TimerAction(period=4.0, actions=[launch_fast_lio2]),
        launch.actions.TimerAction(period=5.0, actions=[launch_octomap_server2]),
        #launch.actions.TimerAction(period=6.0, actions=[launch_scan]),
        launch.actions.TimerAction(period=7.0, actions=[launch_autoaim]),
        launch.actions.TimerAction(period=8.0, actions=[navigation_launch]),
        #launch.actions.TimerAction(period=7.0, actions=[slam_launch]),
        #launch.actions.TimerAction(period=18.0, actions=[launch_judge]),
    ])

  
    rviz_node = launch_ros.actions.Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      arguments=['-d', rviz_config_dir],
      parameters=[{'use_sim_time': use_sim_time}],
      output='log'
     )

   
    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('map', default_value=map_yaml_path, description='Full path to map file to load'),
        DeclareLaunchArgument('params_file', default_value=nav2_param_path, description='Full path to param file to load'),
    ]

    #  LaunchDescription
    return launch.LaunchDescription([
        *declared_arguments,
        static_transform_node,
        rviz_node,
        action_group,
        #launch_judge
    ])
