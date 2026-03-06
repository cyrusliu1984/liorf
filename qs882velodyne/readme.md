

# qs882velodyne_node

## 概述

`qs882velodyne_node` 是一个 ROS 节点，用于将 QS882 激光雷达的点云数据转换为 Velodyne 标准格式。该节点直接使用激光雷达的 `laserid` 字段作为点云的 `ring` 字段，无需回退逻辑，确保了转换的精确性和效率。

## 核心功能

- **数据转换**：将 QS882 点云（包含 `x, y, z, intensity, laserid, point_time`）转换为 Velodyne 格式（`x, y, z, intensity, ring, time`）
- **ring 字段处理**：直接将 `laserid` 作为 `ring` 字段（无回退逻辑）
- **时间字段处理**：将 `point_time`（uint32_t）转换为 0~0.1 秒范围的浮点时间戳
- **调试支持**：可打印详细处理信息（默认开启）

## 依赖

- ROS Noetic
- `roscpp`
- `sensor_msgs`
- `pcl_ros`（可选，用于点云处理）

## 安装

1. 将此节点代码放入 ROS 工作空间的 `src` 目录：
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/cyrusliu1984/liorf/qs882velodyne.git
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
   roslaunch qs882velodyne qs882velodyne.launch
   ```

2. **参数配置**（在 `launch` 文件中或通过命令行）：
   ```xml
   <param name="input_topic" value="/point_raw" />
   <param name="output_topic" value="/czlidar_points" />
   <param name="num_rings" value="88" />
   <param name="debug_print" value="true" />
   <param name="debug_print_count" value="20" />
   ```

3. **参数说明**：
   | 参数 | 类型 | 默认值 | 说明 |
   |------|------|--------|------|
   | `input_topic` | string | `/point_raw` | 输入点云话题 |
   | `output_topic` | string | `/czlidar_points` | 输出点云话题 |
   | `num_rings` | int | 88 | 激光雷达环数（与QS882硬件一致） |
   | `debug_print` | bool | true | 是否打印调试信息 |
   | `debug_print_count` | int | 20 | 打印的点数（调试用） |

## 输入/输出

### 输入
- **话题**: `/point_raw` (默认)
- **消息类型**: `sensor_msgs/PointCloud2`
- **字段**:
  - `x`, `y`, `z`: 3D坐标 (float)
  - `intensity`: 强度 (float)
  - `laserid`: 激光ID (uint32)
  - `point_time`: 点时间戳 (uint32)

### 输出
- **话题**: `/czlidar_points` (默认)
- **消息类型**: `sensor_msgs/PointCloud2`
- **字段**:
  - `x`, `y`, `z`: 3D坐标 (float)
  - `intensity`: 强度 (float)
  - `ring`: 环号 (uint16)
  - `time`: 时间戳 (float, 0.0~0.1秒)

## 工作原理

1. **ring 字段**：直接将 `laserid` 转换为 `ring`（`uint32` → `uint16`）
2. **time 字段**：
   - 有效 `point_time`：归一化到 0~0.1 秒范围
   - 无效 `point_time`（0）：使用点索引生成均匀分布时间
3. **性能优化**：
   - 避免额外的类型转换和回退逻辑
   - 使用高效的点云迭代器

## 调试信息示例

```
[ INFO] [1680000000.123456]: Point [0] | x=0.123, y=0.456, z=0.789, intensity=1.2 | laserid=5 : ring=5 | point_time=12345 : time=0.006172
[ INFO] [1680000000.123457]: Point [1] | x=0.234, y=0.567, z=0.890, intensity=2.3 | laserid=3 : ring=3 | point_time=67890 : time=0.033945
...
```

## 注意事项

1. **硬件匹配**：确保 `num_rings` 参数与您的 QS882 激光雷达硬件一致（通常为 88）
2. **时间处理**：`point_time` 值范围需在 0~2000000 之间，超出范围将被截断
3. **点云格式**：输入点云必须包含 `laserid` 和 `point_time` 字段
4. **ROS 版本**：仅支持 ROS Noetic（基于 C++11）

## 示例启动命令

```bash
roslaunch qs882velodyne qs882velodyne.launch input_topic:=/raw_points output_topic:=/velodyne_points num_rings:=88 debug_print:=true
```



