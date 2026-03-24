#!/bin/bash
# 发送测试数据到 /cmd_vel 和 /fired_info
# 使用方法: ./send_test_data.sh

v=1.0
angle=0.0
count=0
state=0

echo "开始发送测试数据..."
echo "按 Ctrl+C 停止"
echo "========================================"

while true; do
  # 计算弧度 (0.5度 = 0.017453 弧度)
  rad=$(echo "scale=6; $angle * 0.01745329252" | bc)
  
  # 打印调试信息
  echo "T: $count | V: $v | Deg: $angle | Rad: $rad | State: $state"
  
  # 使用单引号包裹整个JSON字符串,避免bash解析
  # /cmd_vel
  ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: '$v', y: '$v', z: '$v'}, angular: {x: 0, y: 0, z: 0}}' \
    --qos-durability transient-local 2>/dev/null &
  
  # /fired_info  
  ros2 topic pub -1 /fired_info auto_aim_interfaces/msg/FiredInfo \
    '{aim_pitch: '$rad', aim_yaw: '$rad', auto_fire_flag: '$state', state: '$state', pose_state: '$state'}' \
    --qos-durability transient-local 2>/dev/null &
  
  # 更新变量
  v=$(echo "$v + 0.5" | bc)
  angle=$(echo "$angle + 0.5" | bc)
  count=$((count + 1))
  
  # 每3秒切换状态
  if [ $((count % 3)) -eq 0 ]; then
    state=$((1 - state))
    echo ">> [状态切换] -> $state"
  fi
  
  sleep 1
done
