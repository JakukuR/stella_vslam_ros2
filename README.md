## By JakukuR : 
1. The compressed image topics are used instead of the standard image_raw topics, for optimized performance with rosbag files.
2. Real-World Testing: The system has been successfully tested on the D435 chassis with live data from the camera, providing practical real-world results.
---

# Stella SLAM ROS2 Integration

This repository contains scripts to run the **SLAM (Simultaneous Localization and Mapping)** system using **ROS2** with a **RGBD camera**. It includes two main scripts:

* **`run_slam.sh`**: This script is used to run the real-time SLAM with live camera data.
* **`run_slam_offline.sh`**: This script allows you to run SLAM on recorded **rosbag** data, useful for offline processing.

## Prerequisites

Before using the scripts, ensure the following are installed:

1. **ROS2 Humble**:

   * Follow the official [ROS2 installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

2. **Stella V-SLAM**:

   * This project relies on the **Stella V-SLAM** package. You should have it installed and configured in your ROS2 workspace.

3. **ORB Vocabulary File**:

   * The script requires an **ORB vocabulary file** (`orb_vocab.fbow`). Ensure it is available in the workspace.

4. **Camera Setup**:

   * The system is designed to work with a camera setup (e.g., **RGBD cameras**, such as ZED or Realsense).

## Usage

### 1. Real-time SLAM

This script runs the SLAM algorithm on real-time camera data.

```bash
bash run_slam.sh
```

#### Details:

* The script sources the necessary ROS2 environment.
* It publishes transformations (TF) between the camera frame and chassis frame.
* It runs the SLAM process with configuration files (e.g., `chassis_rgbd.yaml`) and a vocabulary file.
* The generated map and trajectory are saved in the `~/ros2_ws/maps` directory with a timestamp.

#### Output:

* **Map**: Saved as a `.msg` file.
* **Trajectory**: Saved as a `.txt` file.

The map is saved in the directory:

```
~/ros2_ws/maps/map_YYYYMMDD_HHMMSS.msg
```

### 2. Offline SLAM with Rosbag Data

This script allows you to run SLAM offline using a **rosbag** file.

```bash
bash run_slam_offline.sh /path/to/rosbag
```

You can optionally specify a custom map name:

```bash
bash run_slam_offline.sh /path/to/rosbag custom_map_name
```

#### Details:

* The script processes a **rosbag** file with camera data (e.g., color and depth images).
* It uses the **`run_slam_offline`** node to run SLAM offline and save the results.
* The map and trajectory are saved in the `~/ros2_ws/maps` directory.

#### Output:

* **Map**: Saved as a `.msg` file.
* **Trajectory**: Saved as a `.txt` file.

After running the script, the map file can be converted to **PCD** format using the following command:

```bash
python3 msg_to_pcd.py ~/ros2_ws/maps/custom_map_name.msg --trajectory
```

For visualization, you can use the **PCL viewer**:

```bash
pcl_viewer ~/ros2_ws/maps/custom_map_name.pcd
```

### File Locations

* **Map Output Directory**: `~/ros2_ws/maps`
* **Vocabulary File**: `/home/wx/ros2_ws/orb_vocab.fbow`
* **Configuration File**: `/home/wx/ros2_ws/src/stella_vslam_ros/config/chassis_rgbd.yaml`

## Notes

* Ensure the correct camera topic names are used in the configuration files.
* If you encounter any issues with **TF transforms**, verify the static transform publisher is running correctly.
* If running offline SLAM, ensure the **rosbag** contains the correct camera data (color and depth images).

---
