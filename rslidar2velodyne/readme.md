

# rslidar2velodyne_node

## 概述

`rslidar2velodyne_node` 是一个 ROS 节点，用于将 RSLIDAR（RS80 激光雷达）的点云数据转换为 Velodyne 标准格式。该节点通过计算点的垂直角度，将点分配到对应的 ring（环），并生成合适的时间戳，使 RSLIDAR 数据能够与 Velodyne 传感器数据兼容。注意：目前此功能并不完善，此代码仅做构思实现！

## 核心功能

- **垂直角度匹配**：使用 RS80 默认垂直角度表，将点的垂直角度匹配到对应的 ring
- **时间戳计算**：基于点索引和扫描周期（scan_period）计算时间戳
- **参数化配置**：可配置输入/输出话题、环数、扫描周期等
- **错误处理**：对无效参数和异常情况进行优雅处理
- **调试支持**：可打印详细处理信息（默认关闭）

## 依赖

- ROS Noetic
- `roscpp`
- `sensor_msgs`
- `pcl_ros`（可选，用于点云处理）

## 安装

1. 将此节点代码放入 ROS 工作空间的 `src` 目录：
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/cyrusliu1984/liorf/rslidar2velodyne.git
   ```

2. 构建工作空间：
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

## 使用

1. **启动节点**：
   ```bash
   roslaunch rslidar2velodyne rslidar2velodyne.launch
   ```

2. **参数配置**（在 `launch` 文件中或通过命令行）：
   ```xml
   <param name="input_topic" value="/point_raw" />
   <param name="output_topic" value="/rslidar_points_velodyne" />
   <param name="num_rings" value="80" />
   <param name="debug_print" value="false" />
   <param name="debug_print_count" value="20" />
   <param name="scan_period" value="0.1" />
   ```

3. **参数说明**：
   | 参数 | 类型 | 默认值 | 说明 |
   |------|------|--------|------|
   | `input_topic` | string | `/point_raw` | 输入点云话题 |
   | `output_topic` | string | `/rslidar_points_velodyne` | 输出点云话题 |
   | `num_rings` | int | 80 | 激光雷达环数（RS80 为 80） |
   | `debug_print` | bool | false | 是否打印调试信息 |
   | `debug_print_count` | int | 20 | 打印的点数（调试用） |
   | `scan_period` | float | 0.1 | 扫描周期（秒） |

## 输入/输出

### 输入
- **话题**: `/point_raw` (默认)
- **消息类型**: `sensor_msgs/PointCloud2`
- **字段**:
  - `x`, `y`, `z`: 3D坐标 (float)
  - `intensity`: 强度 (float)

### 输出
- **话题**: `/rslidar_points_velodyne` (默认)
- **消息类型**: `sensor_msgs/PointCloud2`
- **字段**:
  - `x`, `y`, `z`: 3D坐标 (float)
  - `intensity`: 强度 (float)
  - `ring`: 环号 (uint16)
  - `time`: 时间戳 (float, 0.0~scan_period)

## 工作原理

1. **垂直角度计算**：
   - 通过 `calcPointVertAngle` 计算点的垂直角度
   - 使用 `RS80_DEFAULT_VERT_ANGLES` 表格定义 RS80 激光雷达的默认垂直角度

2. **ring 分配**：
   - 生成垂直角度到 ring 的映射表 (`genRS80Vert2RingMap`)
   - 通过 `matchVertAngle2Ring` 匹配计算的垂直角度到最近的预定义角度
   - 确保 ring 在有效范围内（0~num_rings-1）

3. **time 计算**：
   - `time = (point_index / total_points) * scan_period`
   - 确保时间戳在 0~scan_period 范围内均匀分布

## 调试信息示例

```
[ INFO] [1680000000.123456]: Point [0] | x=0.123, y=0.456, z=0.789, intensity=1.2 | ring=34 | time=0.005000
[ INFO] [1680000000.123457]: Point [1] | x=0.234, y=0.567, z=0.890, intensity=2.3 | ring=28 | time=0.010000
...
```

## 注意事项

1. **硬件匹配**：
   - 确保 `num_rings` 参数与您的 RSLIDAR（RS80）硬件一致（默认为 80）
   - RS80 默认有 80 个环（垂直角度范围 -24.5°~+24.5°）

2. **时间计算**：
   - `scan_period` 参数应与激光雷达的实际扫描周期匹配
   - 通常为 0.1 秒（10Hz）

3. **点云格式**：
   - 输入点云必须包含 `x, y, z, intensity` 字段
   - 不需要 `laserid` 或 `point_time` 字段（与 qs882velodyne 不同）

4. **ROS 版本**：
   - 仅支持 ROS Noetic（基于 C++11）

## 示例启动命令

```bash
roslaunch rslidar2velodyne rslidar2velodyne.launch input_topic:=/raw_points output_topic:=/velodyne_points num_rings:=80 scan_period:=0.1
```

## 贡献

欢迎提交 PR 以改进代码、文档或添加新功能。请确保遵循 ROS 编码规范。

---

