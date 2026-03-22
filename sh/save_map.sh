#!/bin/bash

# 1. 自动获取【脚本文件】所在的绝对路径 (即 .../sentry_nav_3.0.3/sh)
# 无论以后项目文件夹改名还是移动位置，这里都会自动更新
SH_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)

# 2. 通过 ../ 自动定位到脚本目录的上一层，再进入 maps 文件夹
# 这样就锁定了 /home/rm/sentry_all/sentry_nav_3.0.3/maps
MAP_DIR="$(dirname "$SH_DIR")/maps"

# 3. 检查文件夹是否存在，不存在则创建
if [ ! -d "$MAP_DIR" ]; then
    mkdir -p "$MAP_DIR"
fi

# 4. 构造完整的文件路径
filename="$MAP_DIR/map_$(date +"%Y%m%d_%H%M%S")"

echo "地图将保存至项目根目录下的 maps 文件夹: $filename"

# 5. 执行保存
ros2 run nav2_map_server map_saver_cli -t /projected_map -f "$filename" --fmt pgm

#!/bin/bash

## Generate a filename with the current date and time
#filename="./maps/map_$(date +"%Y%m%d_%H%M%S")"

## Run the map_saver_cli command with the dynamic filename
#ros2 run nav2_map_server map_saver_cli -t /projected_map -f "$filename" --fmt pgm

## ros2 service call /map_save std_srvs/srv/Trigger