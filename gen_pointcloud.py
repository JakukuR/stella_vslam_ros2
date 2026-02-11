#!/usr/bin/env python3
"""
稠密点云生成脚本
从 RealSense 的 aligned_depth + color 话题实时生成彩色点云，
累积多帧后保存为 PLY 文件。

用法:
  source /opt/ros/humble/setup.bash
  python3 gen_pointcloud.py                   # 默认累积 50 帧
  python3 gen_pointcloud.py --frames 100      # 累积 100 帧
  python3 gen_pointcloud.py --single          # 只捕获 1 帧
  python3 gen_pointcloud.py --output my.ply   # 指定输出文件名
"""

import argparse
import sys
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class PointCloudGenerator(Node):
    def __init__(self, max_frames, output_path, voxel_size):
        super().__init__('pointcloud_generator')
        self.bridge = CvBridge()
        self.max_frames = max_frames
        self.output_path = output_path
        self.voxel_size = voxel_size

        # 相机内参（从 camera_info 获取）
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.intrinsics_ready = False

        # 数据缓存
        self.latest_depth = None
        self.latest_color = None
        self.depth_lock = threading.Lock()
        self.color_lock = threading.Lock()

        # 累积的点云
        self.all_points = []
        self.all_colors = []
        self.frame_count = 0

        # QoS: 与 RealSense 驱动兼容
        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # 订阅 camera_info 获取内参
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/chassis_depth_cam/aligned_depth_to_color/camera_info',
            self.info_callback,
            qos
        )

        # 订阅 aligned depth
        self.depth_sub = self.create_subscription(
            Image,
            '/chassis_depth_cam/aligned_depth_to_color/image_raw',
            self.depth_callback,
            qos
        )

        # 订阅 color
        self.color_sub = self.create_subscription(
            Image,
            '/chassis_depth_cam/color/image_raw',
            self.color_callback,
            qos
        )

        # 定时器：每 0.5s 尝试生成一帧点云
        self.timer = self.create_timer(0.5, self.process_frame)

        self.get_logger().info(f'点云生成器已启动，目标帧数: {max_frames}')

    def info_callback(self, msg):
        if not self.intrinsics_ready:
            # K 矩阵: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.intrinsics_ready = True
            self.get_logger().info(
                f'相机内参已获取: fx={self.fx:.1f}, fy={self.fy:.1f}, '
                f'cx={self.cx:.1f}, cy={self.cy:.1f}, '
                f'分辨率={msg.width}x{msg.height}'
            )

    def depth_callback(self, msg):
        with self.depth_lock:
            self.latest_depth = msg

    def color_callback(self, msg):
        with self.color_lock:
            self.latest_color = msg

    def process_frame(self):
        if not self.intrinsics_ready:
            self.get_logger().info('等待相机内参...', throttle_duration_sec=2.0)
            return

        # 获取最新帧
        with self.depth_lock:
            depth_msg = self.latest_depth
            self.latest_depth = None
        with self.color_lock:
            color_msg = self.latest_color
            self.latest_color = None

        if depth_msg is None or color_msg is None:
            self.get_logger().info('等待深度/彩色图像...', throttle_duration_sec=2.0)
            return

        # 转换图像
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return

        # 确保尺寸一致
        if depth_image.shape[:2] != color_image.shape[:2]:
            import cv2
            color_image = cv2.resize(color_image, (depth_image.shape[1], depth_image.shape[0]))

        # 深度图转点云
        points, colors = self.depth_to_pointcloud(depth_image, color_image)

        if len(points) > 0:
            self.all_points.append(points)
            self.all_colors.append(colors)
            self.frame_count += 1
            total_pts = sum(len(p) for p in self.all_points)
            self.get_logger().info(
                f'帧 {self.frame_count}/{self.max_frames} | '
                f'本帧点数: {len(points)} | 累积总点数: {total_pts}'
            )

        if self.frame_count >= self.max_frames:
            self.save_pointcloud()
            raise SystemExit(0)

    def depth_to_pointcloud(self, depth_image, color_image):
        """将 depth + color 转换为 3D 点云"""
        h, w = depth_image.shape[:2]

        # 生成像素坐标网格
        u = np.arange(w)
        v = np.arange(h)
        u, v = np.meshgrid(u, v)

        # 深度值（单位: 毫米 → 米）
        z = depth_image.astype(np.float32) / 1000.0

        # 过滤无效深度 (0 或过远)
        valid = (z > 0.1) & (z < 5.0)

        z_valid = z[valid]
        u_valid = u[valid].astype(np.float32)
        v_valid = v[valid].astype(np.float32)

        # 反投影到 3D
        x = (u_valid - self.cx) * z_valid / self.fx
        y = (v_valid - self.cy) * z_valid / self.fy

        points = np.stack([x, y, z_valid], axis=-1)

        # 颜色 (BGR → RGB, 归一化到 0-255)
        colors = color_image[valid][:, ::-1]  # BGR → RGB

        return points, colors

    def voxel_downsample(self, points, colors):
        """体素下采样，减少点数"""
        if self.voxel_size <= 0 or len(points) == 0:
            return points, colors

        # 计算体素索引
        voxel_indices = np.floor(points / self.voxel_size).astype(np.int32)

        # 用字典去重，保留每个体素内第一个点
        seen = {}
        keep = []
        for i in range(len(voxel_indices)):
            key = tuple(voxel_indices[i])
            if key not in seen:
                seen[key] = True
                keep.append(i)

        keep = np.array(keep)
        return points[keep], colors[keep]

    def save_pointcloud(self):
        """保存为 PLY 文件"""
        if not self.all_points:
            self.get_logger().error('没有点云数据可保存！')
            return

        self.get_logger().info('正在合并和下采样...')
        all_points = np.vstack(self.all_points)
        all_colors = np.vstack(self.all_colors)

        # 体素下采样
        if self.voxel_size > 0:
            before = len(all_points)
            all_points, all_colors = self.voxel_downsample(all_points, all_colors)
            self.get_logger().info(
                f'下采样: {before} → {len(all_points)} 点 '
                f'(体素大小: {self.voxel_size}m)'
            )

        # 写入 PLY 文件
        self.get_logger().info(f'保存 {len(all_points)} 个点到 {self.output_path} ...')
        self.write_ply(self.output_path, all_points, all_colors)
        self.get_logger().info(f'点云已保存: {self.output_path}')

    def write_ply(self, filepath, points, colors):
        """写入 PLY 格式文件"""
        n = len(points)
        header = (
            "ply\n"
            "format ascii 1.0\n"
            f"element vertex {n}\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "property uchar red\n"
            "property uchar green\n"
            "property uchar blue\n"
            "end_header\n"
        )

        with open(filepath, 'w') as f:
            f.write(header)
            for i in range(n):
                x, y, z = points[i]
                r, g, b = colors[i].astype(int)
                f.write(f"{x:.4f} {y:.4f} {z:.4f} {r} {g} {b}\n")


def main():
    parser = argparse.ArgumentParser(description='从 RealSense 话题生成稠密彩色点云')
    parser.add_argument('--frames', type=int, default=50,
                        help='累积帧数 (默认: 50)')
    parser.add_argument('--single', action='store_true',
                        help='只捕获 1 帧')
    parser.add_argument('--output', '-o', type=str, default='',
                        help='输出 PLY 文件路径 (默认: maps/pointcloud_时间戳.ply)')
    parser.add_argument('--voxel', type=float, default=0.01,
                        help='体素下采样大小，单位米 (默认: 0.01, 即1cm; 0=不下采样)')
    args = parser.parse_args()

    if args.single:
        args.frames = 1

    # 输出路径
    if not args.output:
        import os
        map_dir = '/home/realman/ros2_ws/maps'
        os.makedirs(map_dir, exist_ok=True)
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        args.output = f'{map_dir}/pointcloud_{timestamp}.ply'

    rclpy.init()
    node = PointCloudGenerator(args.frames, args.output, args.voxel)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        node.get_logger().info('用户中断，保存已捕获的数据...')
        node.save_pointcloud()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
