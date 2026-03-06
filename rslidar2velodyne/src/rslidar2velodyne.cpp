// 修复1：条件编译_GNU_SOURCE，避免重复定义
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

// 核心头文件（适配Noetic）
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cmath>
#include <stdexcept>
#include <limits>
#include <vector>
#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <cfloat>

// RS80默认垂直角
const float RS80_DEFAULT_VERT_ANGLES[80] = {
  15.0f,  14.0f,  13.0f,  12.0f,  11.0f,  10.0f,   9.0f,   8.0f,   7.0f,   6.0f,
  5.0f,    4.0f,   3.0f,   2.0f,   1.0f,   0.0f,  -1.0f,  -2.0f,  -3.0f,  -4.0f,
  -5.0f,  -6.0f,  -7.0f,  -8.0f,  -9.0f, -10.0f, -11.0f, -12.0f, -13.0f, -14.0f,
  -15.0f, -16.0f, -17.0f, -18.0f, -19.0f, -20.0f, -21.0f, -22.0f, -23.0f, -24.0f,
  14.5f,  13.5f,  12.5f,  11.5f,  10.5f,   9.5f,   8.5f,   7.5f,   6.5f,   5.5f,
  4.5f,    3.5f,   2.5f,   1.5f,   0.5f,  -0.5f,  -1.5f,  -2.5f,  -3.5f,  -4.5f,
  -5.5f,  -6.5f,  -7.5f,  -8.5f,  -9.5f, -10.5f, -11.5f, -12.5f, -13.5f, -14.5f,
  -15.5f, -16.5f, -17.5f, -18.5f, -19.5f, -20.5f, -21.5f, -22.5f, -23.5f, -24.5f
};

// 生成RS80垂直角→ring映射表
std::vector<std::pair<float, uint16_t>> genRS80Vert2RingMap() {
  std::vector<std::pair<float, uint16_t>> vert_ring_pairs;
  vert_ring_pairs.reserve(80);
  for (uint16_t i = 0; i < 80; ++i) {
    vert_ring_pairs.emplace_back(RS80_DEFAULT_VERT_ANGLES[i], i);
  }

  // 修复2：Lambda参数显式指定类型（C++11兼容，替换auto）
  std::sort(vert_ring_pairs.begin(), vert_ring_pairs.end(),
            [](const std::pair<float, uint16_t>& a, const std::pair<float, uint16_t>& b) {
              return a.first < b.first; // 垂直角越小，ring值越小
            });

  std::vector<std::pair<float, uint16_t>> vert2ring;
  vert2ring.reserve(80);
  for (uint16_t ring = 0; ring < 80; ++ring) {
    vert2ring.emplace_back(vert_ring_pairs[ring].first, ring);
  }
  return vert2ring;
}

// 计算点的垂直角
float calcPointVertAngle(float x, float y, float z) {
  const float r = std::sqrt(x * x + y * y + z * z);
  if (r < 1e-6f) {
    return std::nanf(""); // 替代NAN，C++11兼容
  }
  const float vert_angle_rad = std::asin(z / r);
  const float vert_angle_deg = vert_angle_rad * 180.0f / static_cast<float>(M_PI);
  return vert_angle_deg;
}

// 匹配垂直角到ring
uint16_t matchVertAngle2Ring(float calc_vert_angle, 
                             const std::vector<std::pair<float, uint16_t>>& vert2ring) {
  if (std::isnan(calc_vert_angle)) {
    return 0;
  }

  float min_diff = FLT_MAX;
  uint16_t matched_ring = 0;
  for (const auto& pair : vert2ring) {
    const float diff = std::fabs(calc_vert_angle - pair.first);
    if (diff < min_diff) {
      min_diff = diff;
      matched_ring = pair.second;
    }
  }
  return matched_ring;
}

// 核心类
class Rslidar2VelodyneNode
{
public:
    Rslidar2VelodyneNode(ros::NodeHandle& nh) 
      : nh_(nh), vert2ring_map_(genRS80Vert2RingMap())
    {
        // 读取参数
        nh_.param<std::string>("input_topic", input_topic_, "/point_raw");
        nh_.param<std::string>("output_topic", output_topic_, "/czlidar_points");
        nh_.param<int>("num_rings", num_rings_, 88);
        nh_.param<bool>("debug_print", debug_print_, false);
        nh_.param<int>("debug_print_count", debug_print_count_, 20);
        nh_.param<float>("scan_period", scan_period_, 0.1f);

        // 参数校验
        const uint16_t max_uint16 = std::numeric_limits<uint16_t>::max();
        if (num_rings_ <= 0 || num_rings_ > static_cast<int>(max_uint16)) {
            ROS_ERROR("Invalid num_rings: %d (must be 1~%u)", num_rings_, max_uint16);
            throw std::invalid_argument("num_rings out of valid range");
        }
        if (debug_print_count_ < 0) {
            debug_print_count_ = 0;
            ROS_WARN("debug_print_count set to 0 (invalid negative value)");
        }
        if (scan_period_ <= 0.0f || scan_period_ > 1.0f) {
            scan_period_ = 0.1f;
            ROS_WARN("scan_period set to default 0.1s (invalid value)");
        }

        // 打印信息
        ROS_INFO("=== RSLIDAR to Velodyne XYZIRT Converter (Noetic) ===");
        ROS_INFO("Input Topic: %s | Output Topic: %s", input_topic_.c_str(), output_topic_.c_str());
        ROS_INFO("Laser Rings: %d | Scan Period: %.2fs", num_rings_, scan_period_);

        // 订阅/发布器
        subscription_ = nh_.subscribe(
            input_topic_, 10, 
            &Rslidar2VelodyneNode::pointcloudCallback, this);
        publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);
    }

private:
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input_msg)
    {
        try {
            // 初始化输出点云
            sensor_msgs::PointCloud2 output_msg;
            output_msg.header = input_msg->header;
            output_msg.height = input_msg->height;
            output_msg.width = input_msg->width;
            output_msg.is_bigendian = input_msg->is_bigendian;
            output_msg.is_dense = true;

            // 设置XYZIRT字段
            sensor_msgs::PointCloud2Modifier modifier(output_msg);
            modifier.setPointCloud2Fields(6,
                "x", 1, sensor_msgs::PointField::FLOAT32,
                "y", 1, sensor_msgs::PointField::FLOAT32,
                "z", 1, sensor_msgs::PointField::FLOAT32,
                "intensity", 1, sensor_msgs::PointField::FLOAT32,
                "ring", 1, sensor_msgs::PointField::UINT16,
                "time", 1, sensor_msgs::PointField::FLOAT32
            );
            modifier.resize(input_msg->height * input_msg->width);

            // 迭代器
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*input_msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*input_msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*input_msg, "z");
            sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*input_msg, "intensity");

            sensor_msgs::PointCloud2Iterator<float> iter_x_out(output_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y_out(output_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z_out(output_msg, "z");
            sensor_msgs::PointCloud2Iterator<float> iter_intensity_out(output_msg, "intensity");
            sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_out(output_msg, "ring");
            sensor_msgs::PointCloud2Iterator<float> iter_time_out(output_msg, "time");

            // 遍历点云
            const size_t total_points = input_msg->height * input_msg->width;
            if (debug_print_) {
                ROS_INFO("\n=== Processing Frame (Total Points: %zu) ===", total_points);
            }

            for (size_t i = 0; i < total_points; 
                 ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity,
                 ++iter_x_out, ++iter_y_out, ++iter_z_out, ++iter_intensity_out,
                 ++iter_ring_out, ++iter_time_out)
            {
                // 复制基础数据
                *iter_x_out = *iter_x;
                *iter_y_out = *iter_y;
                *iter_z_out = *iter_z;
                *iter_intensity_out = *iter_intensity;

                // 计算ring
                const float vert_angle = calcPointVertAngle(*iter_x, *iter_y, *iter_z);
                uint16_t ring = matchVertAngle2Ring(vert_angle, vert2ring_map_);
                
                // 范围限制
                const uint16_t num_rings_uint = static_cast<uint16_t>(num_rings_);
                if (ring >= num_rings_uint) {
                    ring = num_rings_uint - 1;
                    ROS_WARN_THROTTLE(1.0, "Ring %u exceeds num_rings %d (clamped)", ring, num_rings_);
                }
                *iter_ring_out = ring;
            
                // 计算time
                const float time = (static_cast<float>(i) / static_cast<float>(total_points)) * scan_period_;
                *iter_time_out = time;

                // 调试打印
                if (debug_print_ && i < static_cast<size_t>(debug_print_count_)) {
                    ROS_INFO(
                        "Point [%zu] | x=%.3f, y=%.3f, z=%.3f, intensity=%.1f | ring=%u | time=%.6f",
                        i, *iter_x, *iter_y, *iter_z, *iter_intensity, ring, time
                    );
                }
            }

            // 发布点云
            publisher_.publish(output_msg);

            if (debug_print_) {
                ROS_INFO("=== Frame Processing Completed ===\n");
            }

        } catch (const std::invalid_argument& e) {
            ROS_ERROR("Invalid argument error: %s", e.what());
        } catch (const std::out_of_range& e) {
            ROS_ERROR("Out of range error: %s", e.what());
        } catch (const std::exception& e) {
            ROS_ERROR("Standard error: %s", e.what());
        } catch (...) {
            ROS_ERROR("Unknown error occurred!");
        }
    }

    // 成员变量
    ros::NodeHandle nh_;
    ros::Subscriber subscription_;
    ros::Publisher publisher_;
    std::string input_topic_;
    std::string output_topic_;
    int num_rings_;
    bool debug_print_;
    int debug_print_count_;
    float scan_period_;
    std::vector<std::pair<float, uint16_t>> vert2ring_map_;
};

// 主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "rslidar2velodyne_node");
    ros::NodeHandle nh("~");

    try {
        Rslidar2VelodyneNode converter_node(nh);
        ROS_INFO("RSLIDAR to Velodyne converter node started successfully (ROS Noetic)!");
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Node initialization failed: %s", e.what());
        return 1;
    }
    return 0;
}

