import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration 

def generate_launch_description():

  config_path = os.path.join(
      get_package_share_directory('sentry_startup'), 'params') 
  
  twist2chassis_cmd_node=Node(
    package='cmd_chassis',
    executable='twist2chassis_cmd',
    output='screen',
  )
  
  # 已删除 fake_joint 和 twist_transformer
  # odom->base_link 由机器人里程计/IMU 动态发布

  # rot_imu=Node(
  #   package='cmd_chassis',
  #   executable='rot_imu',
  #   output='screen'
  # )
 
  start_extra_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('dm_imu'),
                'launch',
                'run_without_rviz.launch.py'  # 注意：这是 .py 文件，应该用 PythonLaunchDescriptionSource
            )
        )
    )

  sentry_description = IncludeLaunchDescription(
     PythonLaunchDescriptionSource([os.path.join(
       #get_package_share_directory('sentry_description'), 'launch', 'view_model.launch.py')])
       get_package_share_directory('sentry_description'), 'launch', 'display_robot.launch.py')])
   )

  # mid360
  mid360_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
          get_package_share_directory('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360_launch.py')])
  )

  # icp relocalization - 使用静态 map->odom 变换
  # map → odom 静态变换 (根据初始坐标设置)
  map_odom_trans = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='map_to_odom',
      # x, y, z, roll, pitch, yaw, parent_frame, child_frame
      arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'map', 'odom']
  )

  保留ICP节点用于点云匹配验证(可选,不发布TF)
  icp_node = Node(
      package='icp_relocalization',
      executable='icp_node',
      name='icp_node',
      output='screen',
      parameters=[
          {'initial_x':0.0},
          {'initial_y':0.0},
          {'initial_z':0.0},
          {'initial_a':0.0},
          {'map_voxel_leaf_size':0.5},
          {'cloud_voxel_leaf_size':0.3},
          {'map_frame_id':'map'},
          {'solver_max_iter':100},
          {'max_correspondence_distance':0.1},
          {'RANSAC_outlier_rejection_threshold':0.5},
          {'map_path':'/home/rm/sentry_all/sentry_nav_3.0.3/PCD/tset.pcd'},
          {'fitness_score_thre':0.9},
          {'converged_count_thre':40},
          {'pcl_type':'livox'},
      ],
  )
  
  # fast-lio localization   
  fast_lio_param = os.path.join(
      config_path, 'fast_lio_relocalization_param.yaml')
  fast_lio_node = Node(
      package='fast_lio',
      executable='fastlio_mapping',
      parameters=[
          fast_lio_param
      ],
      output='screen',
      #remappings=[('/Odometry','/state_estimation')]
  )
        
  rviz_config_file = os.path.join(
    get_package_share_directory('sentry_startup'), 'rviz', 'loam_livox.rviz')
  start_rviz = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_file,'--ros-args', '--log-level', 'warn'],
    output='screen'
  )

  delayed_start_lio = TimerAction(
    period=5.0,
    actions=[
      icp_node,
      fast_lio_node
    ]
  )

  ld = LaunchDescription()

  ld.add_action(twist2chassis_cmd_node)
  # ld.add_action(fake_joint_node)  # 已删除
  # ld.add_action(twist_transformer_node)  # 已删除
  # ld.add_action(odom_to_base_link)  # 已删除,odom->base_link由机器人里程计动态发布
  #ld.add_action(rot_imu)
  ld.add_action(start_extra_imu)
  ld.add_action(mid360_node)
  ld.add_action(sentry_description) # 没接自瞄的时候这个要开
  ld.add_action(map_odom_trans)
  #ld.add_action(icp_node)
  ld.add_action(start_rviz)
  # ld.add_action(delayed_start_lio)
  ld.add_action(fast_lio_node)

  return ld