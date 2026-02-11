#!/bin/bash
# 离线跑 rosbag 的 SLAM 脚本
# 用法: bash run_slam_offline.sh /path/to/rosbag_dir
#   或: bash run_slam_offline.sh /path/to/rosbag_dir map_name

source /opt/ros/humble/setup.bash
source /home/xw/ros2_ws/install/setup.bash

BAG_PATH="${1:?请指定 rosbag 路径，例如: bash run_slam_offline.sh /path/to/rosbag}"

# 输出目录
MAP_DIR=/home/xw/ros2_ws/maps
mkdir -p "$MAP_DIR"

# 地图文件名（可通过第2个参数自定义）
if [ -n "$2" ]; then
    MAP_NAME="$2"
else
    MAP_NAME="offline_$(date +%Y%m%d_%H%M%S)"
fi

echo "=================================="
echo " Rosbag: $BAG_PATH"
echo " 地图输出: $MAP_DIR/${MAP_NAME}.msg"
echo " 轨迹输出: $MAP_DIR/"
echo "=================================="

ros2 run stella_vslam_ros run_slam_offline \
  -v /home/xw/ros2_ws/orb_vocab.fbow \
  -c /home/xw/ros2_ws/src/stella_vslam_ros/config/chassis_rgbd.yaml \
  -b "$BAG_PATH" \
  --color /chassis_depth_cam/color/image_raw/compressed \
  --depth /chassis_depth_cam/depth/image_raw \
  --mask /home/xw/ros2_ws/src/stella_vslam_ros/config/mask_848x480.png \
  --no-sleep \
  --viewer iridescence_viewer \
  -o "$MAP_DIR/${MAP_NAME}.msg" \
  --eval-log-dir "$MAP_DIR" \
  --ros-args -p publish_tf:=false

echo ""
echo "=== 完成 ==="
echo "地图: $MAP_DIR/${MAP_NAME}.msg"
echo "轨迹: $MAP_DIR/frame_trajectory.txt"
echo ""
echo "转 PCD:  python3 msg_to_pcd.py $MAP_DIR/${MAP_NAME}.msg --trajectory"
echo "可视化:  pcl_viewer $MAP_DIR/${MAP_NAME}.pcd"
