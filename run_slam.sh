#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/wx/ros2_ws/install/setup.bash

# 发布 camera_depth_frame → chassis_depth_cam_link 的单位变换，连接 TF 树
#ros2 run tf2_ros static_transform_publisher \
#  --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
#  --frame-id camera_depth_frame \
#  --child-frame-id chassis_depth_cam_link &
#STATIC_TF_PID=$!

# 等待 static TF 发布就绪
#sleep 1

# 输出目录
MAP_DIR=/home/xw/ros2_ws/maps
mkdir -p "$MAP_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

ros2 run stella_vslam_ros run_slam \
  -v /home/wx/ros2_ws/orb_vocab.fbow \
  -c /home/wx/ros2_ws/src/stella_vslam_ros/config/chassis_rgbd.yaml \
  --mask /home/wx/ros2_ws/src/stella_vslam_ros/config/mask_848x480.png \
  --viewer iridescence_viewer \
  -o "$MAP_DIR/map_${TIMESTAMP}.msg" \
  --eval-log-dir "$MAP_DIR" \
  --ros-args --params-file /home/wx/ros2_ws/src/stella_vslam_ros/config/param.yaml \
  -r camera/color/image_raw:=/chassis_depth_cam/color/image_raw \
  -r camera/depth/image_raw:=/chassis_depth_cam/depth/image_raw

echo ""
echo "=== 地图已保存到: $MAP_DIR/map_${TIMESTAMP}.msg ==="
echo "=== 轨迹已保存到: $MAP_DIR/frame_trajectory.txt ==="

# SLAM 退出后清理后台进程
kill $STATIC_TF_PID 2>/dev/null
