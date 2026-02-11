#!/usr/bin/env python3
"""
将 stella_vslam 的 .msg 地图文件转换为 PCD/PLY 点云文件，并可选可视化。

用法:
  python3 msg_to_pcd.py maps/map_xxx.msg                    # 转为 PCD + PLY
  python3 msg_to_pcd.py maps/map_xxx.msg --visualize        # 转换并用 pcl_viewer 可视化
  python3 msg_to_pcd.py maps/map_xxx.msg -o my_map          # 指定输出前缀 (my_map.pcd, my_map.ply)
  python3 msg_to_pcd.py maps/map_xxx.msg --trajectory       # 同时导出关键帧轨迹
"""

import argparse
import struct
import sys
import os
import subprocess
import numpy as np

try:
    import msgpack
except ImportError:
    print("需要安装 msgpack: pip3 install msgpack")
    sys.exit(1)


def load_msg_file(filepath):
    """加载 stella_vslam 的 .msg (MessagePack) 地图文件"""
    print(f"加载地图文件: {filepath}")
    with open(filepath, 'rb') as f:
        data = msgpack.unpack(f, raw=False)
    return data


def extract_landmarks(data):
    """提取所有 landmark 的 3D 坐标"""
    landmarks = data.get('landmarks', {})
    print(f"共有 {len(landmarks)} 个 3D 路标点")

    points = []
    for lm_id, lm in landmarks.items():
        pos = lm.get('pos_w')
        if pos is not None and len(pos) == 3:
            points.append(pos)

    points = np.array(points, dtype=np.float32)
    print(f"有效点数: {len(points)}")

    if len(points) > 0:
        print(f"坐标范围:")
        print(f"  X: [{points[:,0].min():.3f}, {points[:,0].max():.3f}]")
        print(f"  Y: [{points[:,1].min():.3f}, {points[:,1].max():.3f}]")
        print(f"  Z: [{points[:,2].min():.3f}, {points[:,2].max():.3f}]")

    return points


def quat_to_rotation_matrix(qx, qy, qz, qw):
    """四元数 → 3x3 旋转矩阵"""
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx**2 + qz**2),  2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),      1 - 2*(qx**2 + qy**2)]
    ])
    return R


def extract_keyframe_trajectory(data):
    """提取关键帧轨迹 (位置 + 朝向)"""
    keyframes = data.get('keyframes', {})
    print(f"共有 {len(keyframes)} 个关键帧")

    trajectory = []
    for kf_id, kf in sorted(keyframes.items(), key=lambda x: int(x[0])):
        rot_cw = kf.get('rot_cw')  # 四元数 [x, y, z, w] (world to camera)
        trans_cw = kf.get('trans_cw')  # 平移向量 [tx, ty, tz]
        ts = kf.get('ts', 0.0)

        if rot_cw is not None and trans_cw is not None:
            # 四元数 → 旋转矩阵
            qx, qy, qz, qw = rot_cw
            R = quat_to_rotation_matrix(qx, qy, qz, qw)
            t = np.array(trans_cw).reshape(3, 1)
            # 相机在世界坐标系的位置: pos_w = -R_cw^T * t_cw
            pos_w = -R.T @ t
            trajectory.append({
                'id': kf_id,
                'ts': ts,
                'pos': pos_w.flatten(),
                'quat': [qx, qy, qz, qw]
            })

    return trajectory


def write_pcd(filepath, points):
    """写入 PCD 格式文件 (ASCII)"""
    n = len(points)
    with open(filepath, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {n}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {n}\n")
        f.write("DATA ascii\n")
        for p in points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

    print(f"PCD 已保存: {filepath} ({n} 点)")


def write_pcd_binary(filepath, points):
    """写入 PCD 格式文件 (Binary, 更快更小)"""
    n = len(points)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )

    with open(filepath, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(points.astype(np.float32).tobytes())

    size = os.path.getsize(filepath)
    print(f"PCD (binary) 已保存: {filepath} ({n} 点, {size/1024:.1f} KB)")


def write_ply(filepath, points):
    """写入 PLY 格式文件"""
    n = len(points)
    with open(filepath, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {n}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for p in points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

    print(f"PLY 已保存: {filepath} ({n} 点)")


def write_trajectory_tum(filepath, trajectory):
    """写入 TUM 格式轨迹文件"""
    with open(filepath, 'w') as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for kf in trajectory:
            ts = kf['ts']
            x, y, z = kf['pos']
            # 反转 camera-to-world 的四元数
            qx, qy, qz, qw = kf.get('quat', [0, 0, 0, 1])
            f.write(f"{ts:.6f} {x:.6f} {y:.6f} {z:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")

    print(f"轨迹已保存: {filepath} ({len(trajectory)} 帧)")


def write_trajectory_as_pcd(filepath, trajectory):
    """将轨迹点保存为 PCD，方便一起可视化"""
    points = np.array([kf['pos'] for kf in trajectory], dtype=np.float32)
    write_pcd(filepath, points)


def visualize_pcd(pcd_path):
    """用 pcl_viewer 可视化"""
    # 检查 pcl_viewer 是否可用
    viewer = None
    for cmd in ['pcl_viewer', 'pcl_viewer_release']:
        try:
            subprocess.run([cmd, '--help'], capture_output=True, timeout=3)
            viewer = cmd
            break
        except (FileNotFoundError, subprocess.TimeoutExpired):
            continue

    if viewer:
        print(f"\n启动 {viewer} 可视化...")
        print("操作提示: 滚轮缩放, 左键旋转, Shift+左键平移, 'q' 退出")
        subprocess.run([viewer, pcd_path])
    else:
        print("\n未找到 pcl_viewer。安装方法:")
        print("  sudo apt install pcl-tools")
        print(f"\n你也可以将 {pcd_path} 复制到 PC 上用以下工具打开:")
        print("  - CloudCompare (推荐, 跨平台)")
        print("  - MeshLab")
        print("  - PCL viewer")


def main():
    parser = argparse.ArgumentParser(
        description='将 stella_vslam 的 .msg 地图转换为 PCD/PLY 点云')
    parser.add_argument('input', help='.msg 地图文件路径')
    parser.add_argument('-o', '--output', default='',
                        help='输出文件前缀 (默认: 与输入同名)')
    parser.add_argument('--visualize', '-V', action='store_true',
                        help='转换后用 pcl_viewer 可视化')
    parser.add_argument('--trajectory', '-t', action='store_true',
                        help='同时导出关键帧轨迹')
    parser.add_argument('--binary', action='store_true',
                        help='使用二进制 PCD 格式 (更小更快)')
    parser.add_argument('--format', choices=['pcd', 'ply', 'both'],
                        default='both', help='输出格式 (默认: both)')
    args = parser.parse_args()

    if not os.path.exists(args.input):
        print(f"错误: 文件不存在: {args.input}")
        sys.exit(1)

    # 输出前缀
    if args.output:
        out_prefix = args.output
    else:
        out_prefix = os.path.splitext(args.input)[0]

    # 加载地图
    data = load_msg_file(args.input)

    # 提取 landmark 点
    points = extract_landmarks(data)
    if len(points) == 0:
        print("没有 landmark 点可导出！")
        sys.exit(1)

    # 保存点云
    pcd_path = None
    if args.format in ('pcd', 'both'):
        pcd_path = out_prefix + '.pcd'
        if args.binary:
            write_pcd_binary(pcd_path, points)
        else:
            write_pcd(pcd_path, points)

    if args.format in ('ply', 'both'):
        write_ply(out_prefix + '.ply', points)

    # 导出轨迹
    if args.trajectory:
        trajectory = extract_keyframe_trajectory(data)
        if trajectory:
            write_trajectory_tum(out_prefix + '_trajectory.txt', trajectory)
            write_trajectory_as_pcd(out_prefix + '_trajectory.pcd', trajectory)
            print(f"\n轨迹点也已保存为 PCD，可与地图叠加可视化:")
            print(f"  pcl_viewer {out_prefix}.pcd {out_prefix}_trajectory.pcd")

    # 可视化
    if args.visualize and pcd_path:
        visualize_pcd(pcd_path)
    elif not args.visualize:
        print(f"\n可视化命令:")
        if pcd_path:
            print(f"  pcl_viewer {pcd_path}")
        print(f"  # 或复制到 PC 上用 CloudCompare / MeshLab 打开")


if __name__ == '__main__':
    main()
