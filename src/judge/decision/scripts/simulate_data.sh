#!/bin/bash
# Simulation script for rm_decision behavior tree testing
# Publishes mock data to /serial_packet and /target topics

# Source the workspace first
source ~/sentry_alles/sentry_all/sentry_nav_3.0.3/install/setup.bash

echo "Starting simulation data publishers..."
echo "Publishing pose_state: 1=defense(低血量), 0=attack(有目标), 2=move(其他)"
echo "按 Ctrl+C 停止"

# Initial state
robot_hp=500
tracking=false
pose_state=2

# Publish initial state
ros2 topic pub --once /serial_packet auto_aim_interfaces/msg/SerialPacket "
header: 165
detect_color: 0
task_mode: 1
reset_tracker: false
is_play: true
change_target: false
reserved: 0
roll: 0.0
pitch: 0.0
yaw: 0.0
robot_hp: 500
game_time: 1
checksum: 0
pose_state: 2
"

ros2 topic pub --once /target auto_aim_interfaces/msg/Target "
tracking: false
id: ''
armors_num: 0
position: {x: 0.0, y: 0.0, z: 0.0}
velocity: {x: 0.0, y: 0.0, z: 0.0}
yaw: 0.0
v_yaw: 0.0
radius_1: 0.0
radius_2: 0.0
dz: 0.0
"

echo "开始循环发布数据..."

# Simulate scenario changes
case_num=0

while true; do
    case_num=$((case_num + 1))
    
    # Scenario 1: Normal - move (0-10秒)
    if [ $case_num -le 10 ]; then
        robot_hp=500
        tracking=false
        pose_state=2
        echo "[$case_num] 状态: HP=$robot_hp, tracking=$tracking → pose_state=$pose_state (移动)"
    # Scenario 2: Tracking detected - attack (10-20秒)
    elif [ $case_num -le 20 ]; then
        robot_hp=400
        tracking=true
        pose_state=0
        echo "[$case_num] 状态: HP=$robot_hp, tracking=$tracking → pose_state=$pose_state (攻击)"
    # Scenario 3: Low HP - defense (20-30秒)
    elif [ $case_num -le 30 ]; then
        robot_hp=80
        tracking=true
        pose_state=1
        echo "[$case_num] 状态: HP=$robot_hp, tracking=$tracking → pose_state=$pose_state (防御)"
    # Scenario 4: No tracking - move (30-40秒)
    elif [ $case_num -le 40 ]; then
        robot_hp=300
        tracking=false
        pose_state=2
        echo "[$case_num] 状态: HP=$robot_hp, tracking=$tracking → pose_state=$pose_state (移动)"
    # Reset
    else
        case_num=0
        continue
    fi
    
    # Publish SerialPacket
    ros2 topic pub /serial_packet auto_aim_interfaces/msg/SerialPacket "
header: 165
detect_color: 0
task_mode: 1
reset_tracker: false
is_play: true
change_target: false
reserved: 0
roll: 0.0
pitch: 0.0
yaw: 0.0
robot_hp: $robot_hp
game_time: 1
checksum: 0
pose_state: $pose_state
" &
    
    # Publish Target
    if [ "$tracking" = true ]; then
        ros2 topic pub /target auto_aim_interfaces/msg/Target "
tracking: true
id: 'hero'
armors_num: 1
position: {x: 1.0, y: 0.5, z: 0.0}
velocity: {x: 0.0, y: 0.0, z: 0.0}
yaw: 0.0
v_yaw: 0.0
radius_1: 0.0
radius_2: 0.0
dz: 0.0
" &
    else
        ros2 topic pub /target auto_aim_interfaces/msg/Target "
tracking: false
id: ''
armors_num: 0
position: {x: 0.0, y: 0.0, z: 0.0}
velocity: {x: 0.0, y: 0.0, z: 0.0}
yaw: 0.0
v_yaw: 0.0
radius_1: 0.0
radius_2: 0.0
dz: 0.0
" &
    fi
    
    sleep 1
done
