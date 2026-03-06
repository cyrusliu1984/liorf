

# LIO-SAM + QS882/RSLIDAR 激光雷达建图系统

![LIO-SAM System Architecture](https://img.shields.io/badge/ROS-Noetic-brightgreen) ![Laser Sensor-QS882/RS80-yellowgreen] ![SLAM-LIO-SAM-blue]

## 概述

本项目是一个完整的激光雷达 SLAM 系统，基于 LIO-SAM (Lidar Inertial Odometry and Mapping) 框架，支持 QS882 和 RSLIDAR RS80 激光雷达。系统包括点云转换节点和 SLAM 算法，可实现高精度的室内/室外建图。

## 系统架构

```
激光雷达数据 → 点云转换节点 → LIO-SAM SLAM 系统 → 生成地图
```

## 支持的激光雷达型号

| 型号 | 环数 (N_SCAN) | 水平分辨率 (Horizon_SCAN) | 点云转换节点 |
|------|---------------|---------------------------|--------------|
| QS882 | 88 | 1200 | `qs882velodyne_node` |
| RSLIDAR RS80 | 80 | 1800 | `rslidar2velodyne_node` |

## 依赖环境

- Ubuntu 20.04 LTS
- ROS Noetic
- CMake 3.10+
- PCL 1.10+
- Eigen 3.3+
- OpenCV 4.2+
- Ceres Solver
- GTSAM

## 安装步骤

### 1. 安装 ROS Noetic

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E33F600C6C9A3A3F47427A5A7271D728F98
sudo apt update
sudo apt install ros-noetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. 安装系统依赖

```bash
sudo apt-get install -y build-essential libtool libgflags-dev libgoogle-glog-dev libgtest-dev \
    libatlas-base-dev libboost-all-dev libeigen3-dev libopencv-dev libqhull-dev \
    libsqlite3-dev libusb-1.0-0-dev libvtk7-dev libyaml-cpp-dev libpcre3-dev \
    libtbb-dev libopenexr-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev \
    libxcomposite-dev libxext-dev libxi-dev libxkbcommon-dev libgl1-mesa-dev libglu1-mesa-dev \
    libfreetype6-dev libfontconfig1-dev libssl-dev libcurl4-openssl-dev
```

### 3. 安装 Ceres Solver

```bash
cd ~
git clone https://github.com/ceres-solver/ceres-solver.git
cd ceres-solver
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### 4. 安装 GTSAM

```bash
cd ~
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout gtsam-4.2.0
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/gtsam_install
make -j$(nproc)
sudo make install
```

### 5. 创建 ROS 工作空间并安装项目

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/your-repo/liorf.git
git clone https://github.com/your-repo/qs882velodyne.git
git clone https://github.com/your-repo/rslidar2velodyne.git
cd ~/catkin_ws
catkin_make_isolated --force-cmake
source devel_isolated/setup.bash
```

## 点云转换节点

### 1. QS882 点云转换

`qs882velodyne_node` 将 QS882 激光雷达的点云数据转换为 Velodyne 标准格式。

#### 参数配置 (默认 `params.yaml`)

```yaml
input_topic: "/point_raw"           # 输入点云话题 (来自QS882)
output_topic: "/czlidar_points"     # 输出点云话题 (LIO-SAM输入)
num_rings: 88                      # QS882有88个环
debug_print: true                  # 是否打印调试信息
debug_print_count: 20              # 调试打印点数
```

#### 启动方式

```bash
roslaunch qs882velodyne qs882velodyne.launch
```

#### 调试输出示例

```
[ INFO] [1680000000.123456]: Point [0] | x=0.123, y=0.456, z=0.789, intensity=1.2 | laserid=5 : ring=5 | point_time=12345 : time=0.006172
```

### 2. RSLIDAR RS80 点云转换

`rslidar2velodyne_node` 将 RSLIDAR RS80 激光雷达的点云数据转换为 Velodyne 标准格式。

#### 参数配置 (默认 `params.yaml`)

```yaml
input_topic: "/point_raw"           # 输入点云话题 (来自RS80)
output_topic: "/rslidar_points_velodyne" # 输出点云话题 (LIO-SAM输入)
num_rings: 80                      # RS80有80个环
debug_print: false                 # 默认关闭调试
debug_print_count: 20              # 调试打印点数
scan_period: 0.1                   # 扫描周期 (10Hz)
```

#### 启动方式

```bash
roslaunch rslidar2velodyne rslidar2velodyne.launch
```

#### 调试输出示例

```
[ INFO] [1680000000.123456]: Point [0] | x=0.123, y=0.456, z=0.789, intensity=1.2 | ring=34 | time=0.005000
```

## LIO-SAM (liorf) SLAM

### 参数配置 (默认 `params.yaml`)

```yaml
# Topics
pointCloudTopic: "/czlidar_points"  # Point cloud data (must match conversion node output)
imuTopic: "imu"                     # IMU data
odomTopic: "odometry/imu"           # IMU pre-preintegration odometry
gpsTopic: "gps/fix"                 # GPS odometry topic

# Frames
lidarFrame: "czlidar"               # Lidar frame name
baselinkFrame: "base_link"          # Base link frame name
odometryFrame: "odom"               # Odometry frame name
mapFrame: "map"                     # Map frame name

# GPS Settings
useImuHeadingInitialization: true   # Use GPS data for heading initialization
useGpsElevation: true               # Use GPS elevation data
gpsCovThreshold: 2.0                # GPS covariance threshold
poseCovThreshold: 25.0              # Pose covariance threshold

# Export settings
savePCD: true                       # Save PCD files
savePCDDirectory: "/文档/liorf/PCD/" # Directory to save PCD files (ends with "/")

# Sensor Settings
sensor: velodyne                    # Lidar sensor type
N_SCAN: 88                          # Number of lidar channels (QS882: 88, RS80: 80)
Horizon_SCAN: 1200                  # Lidar horizontal resolution
downsampleRate: 1                   # Downsample rate
point_filter_num: 1                 # Point filter number
lidarMinRange: 3.0                  # Minimum lidar range
lidarMaxRange: 200.0                # Maximum lidar range

# IMU Settings
imuType: 0                          # 0: 6-axis, 1: 9-axis
imuRate: 50.0                       # IMU rate
imuAccNoise: 3.9939570888238808e-03
imuGyrNoise: 1.5636343949698187e-03
imuAccBiasN: 6.4356659353532566e-05
imuGyrBiasN: 3.5640318696367613e-05
imuGravity: 9.80511
imuRPYWeight: 0.01

# Extrinsics: T_lb (lidar -> imu)
extrinsicTrans: [0.0, 0.0, 0.0]
extrinsicRot: [1, 0, 0,
                0, 1, 0,
                0, 0, 1]

# Voxel filter parameters
mappingSurfLeafSize: 0.4            # Voxel size for mapping

# Loop closure
loopClosureEnableFlag: true         # Enable loop closure
loopClosureFrequency: 1.0           # Loop closure frequency
historyKeyframeSearchRadius: 15.0   # Keyframe search radius (meters)
historyKeyframeSearchTimeDiff: 30.0 # Keyframe search time difference (seconds)
historyKeyframeSearchNum: 25        # Number of history keyframes
historyKeyframeFitnessScore: 0.3    # ICP fitness score threshold

# Visualization
globalMapVisualizationSearchRadius: 1000.0  # Global map visualization radius (meters)
globalMapVisualizationPoseDensity: 10.0    # Keyframe density (meters)
globalMapVisualizationLeafSize: 1.0        # Point cloud density (meters)
```

### 启动方式

```bash
roslaunch liorf liorf.launch
```

## 使用流程

1. **启动点云转换节点** (根据激光雷达型号选择):

```bash
# QS882 激光雷达
roslaunch qs882velodyne qs882velodyne.launch

# 或 RSLIDAR RS80 激光雷达
roslaunch rslidar2velodyne rslidar2velodyne.launch
```

2. **启动 LIO-SAM SLAM**:

```bash
roslaunch liorf liorf.launch
```

3. **(可选) 启动 GPS 导航**:

```bash
roslaunch navsat navsat.launch
```

## 参数配置指南

### 1. 点云转换节点参数

| 参数 | 说明 | 建议值 | 适用雷达 |
|------|------|--------|----------|
| `input_topic` | 输入点云话题 | `/point_raw` | 所有 |
| `output_topic` | 输出点云话题 | `/czlidar_points` | QS882`/rslidar_points_velodyne` (RS80) |
| `num_rings` | 激光雷达环数 | 88 (QS882)80 (RS80) | 根据雷达型号 |
| `debug_print` | 是否打印调试信息 | `true` (开发)`false` (生产) | 所有 |
| `scan_period` | 扫描周期 (秒) | 0.1 (10Hz) | RS80 |

### 2. LIO-SAM 参数

| 参数 | 说明 | 建议值 | 适用场景 |
|------|------|--------|----------|
| `N_SCAN` | 激光雷达环数 | 88 (QS882)80 (RS80) | 根据雷达型号 |
| `lidarMinRange` | 最小激光雷达范围 | 3.0 | 所有 |
| `lidarMaxRange` | 最大激光雷达范围 | 200.0 | 所有 |
| `loopClosureEnableFlag` | 是否启用闭环检测 | `true` | 所有 |
| `historyKeyframeSearchRadius` | 闭环搜索半径 | 15.0 | 所有 |
| `historyKeyframeFitnessScore` | ICP匹配阈值 | 0.3 | 所有 |
| `savePCD` | 是否保存PCD文件 | `true` | 所有 |
| `savePCDDirectory` | PCD保存目录 | `/文档/liorf/PCD/` | 所有 |

## 注意事项

1. **目录路径**：确保 `savePCDDirectory` 中的路径存在且有写入权限。系统会自动创建目录。

2. **激光雷达型号**：根据实际使用的激光雷达型号修改 `N_SCAN` 参数：
   - QS882: `N_SCAN: 88`
   - RS80: `N_SCAN: 80`

3. **点云转换节点**：确保点云转换节点的输出话题与 LIO-SAM 的 `pointCloudTopic` 参数一致：
   - QS882: `pointCloudTopic: "/czlidar_points"`
   - RS80: `pointCloudTopic: "/rslidar_points_velodyne"`

4. **IMU 配置**：确保 IMU 数据正确发布，并与配置的 IMU 类型匹配。

5. **GPS 配置**：如果使用 GPS，需要配置 `navsat` 和 `ekf_gps` 参数，确保 GPS 数据正确转换为坐标。

## 故障排除

### 1. 点云转换节点无法启动

- **检查环境变量**：确保已正确 source 环境
  ```bash
  source devel_isolated/setup.bash
  ```
  
- **检查构建**：确保工作空间正确构建
  ```bash
  cd ~/catkin_ws
  catkin_make_isolated
  ```

### 2. LIO-SAM 无法启动

- **检查点云转换节点**：确保点云转换节点已启动
- **检查话题匹配**：确保 `pointCloudTopic` 与点云转换节点的输出话题一致
- **检查参数**：确保 `N_SCAN` 参数与激光雷达型号匹配

### 3. 地图质量不佳

- **调整范围**：修改 `lidarMinRange` 和 `lidarMaxRange` 过滤无效点
- **优化参数**：调整 `mappingSurfLeafSize` 优化地图精度
- **检查 IMU**：确保 IMU 数据准确且频率匹配

## 项目结构

```
catkin_ws/
├── src/
│   ├── liorf/               # LIO-SAM SLAM 系统
│   ├── qs882velodyne/       # QS882 点云转换节点
│   └── rslidar2velodyne/    # RSLIDAR RS80 点云转换节点
├── build_isolated/          # 编译输出
└── devel_isolated/          # 环境变量
```

## 贡献

欢迎提交 PR 以改进代码、文档或添加新功能。请确保遵循 ROS 编码规范。

## 许可证

本项目采用 MIT 许可证。详情请参阅 [LICENSE](LICENSE) 文件。

---

> **重要提示**：确保在运行前正确配置了所有参数，特别是与激光雷达型号相关的参数（如 `N_SCAN`）。不正确的参数配置可能导致建图失败或质量不佳。
