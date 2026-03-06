#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cmath> // Required for fmod function

class Qs882Velodyne_Node
{
public:
    Qs882Velodyne_Node(ros::NodeHandle& nh) : nh_(nh)
    {
        // Read launch/config parameters (consistent with your launch file)
        nh_.param<std::string>("input_topic", input_topic_, "/point_raw");
        nh_.param<std::string>("output_topic", output_topic_, "/czlidar_points");
        nh_.param<int>("num_rings", num_rings_, 88);
        nh_.param<bool>("debug_print", debug_print_, true);
        nh_.param<int>("debug_print_count", debug_print_count_, 20);

        // Print core logic info (English only)
        ROS_INFO("=== Core Logic: laserid directly used as ring (no fallback logic) ===");
        ROS_INFO("Input Topic: %s | Output Topic: %s", input_topic_.c_str(), output_topic_.c_str());
        ROS_INFO("Laser Count: %d | laserid Type: UINT32 | ring Type: UINT16", num_rings_);
        ROS_INFO("point_time Type: UINT32 (integer) | time output Type: FLOAT32");

        // Create subscriber/publisher
        subscription_ = nh_.subscribe(
            input_topic_, 10, 
            &Qs882Velodyne_Node::pointcloudCallback, this);
        publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);
    }

private:
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input_msg)
    {
        try {
            // 1. Initialize output point cloud (keep header consistent with input)
            sensor_msgs::PointCloud2 output_msg;
            output_msg.header = input_msg->header;
            output_msg.height = input_msg->height;
            output_msg.width = input_msg->width;
            output_msg.is_bigendian = input_msg->is_bigendian;
            output_msg.is_dense = input_msg->is_dense;

            // 2. Set output field structure (core fields only)
            // 修正：time字段类型改为FLOAT32（匹配迭代器类型，避免内存错误）
            sensor_msgs::PointCloud2Modifier modifier(output_msg);
            modifier.setPointCloud2Fields(6,
                "x", 1, sensor_msgs::PointField::FLOAT32,
                "y", 1, sensor_msgs::PointField::FLOAT32,
                "z", 1, sensor_msgs::PointField::FLOAT32,
                "intensity", 1, sensor_msgs::PointField::FLOAT32,
                "ring", 1, sensor_msgs::PointField::UINT16,
                "time", 1, sensor_msgs::PointField::FLOAT32 // 修正：从UINT16改回FLOAT32
            );
            modifier.resize(input_msg->height * input_msg->width);

            // 3. Create iterators (match actual field types)
            // Basic fields (FLOAT32, correct reading)
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*input_msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*input_msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*input_msg, "z");
            sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*input_msg, "intensity");
            // laserid: UINT32 (match actual field type)
            sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_laserid(*input_msg, "laserid");
            // 修正：point_time迭代器类型为UINT32（匹配整数类型）
            sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_point_time(*input_msg, "point_time");

            // Output iterators (类型完全匹配)
            sensor_msgs::PointCloud2Iterator<float> iter_x_out(output_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y_out(output_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z_out(output_msg, "z");
            sensor_msgs::PointCloud2Iterator<float> iter_intensity_out(output_msg, "intensity");
            sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_out(output_msg, "ring");
            sensor_msgs::PointCloud2Iterator<float> iter_time_out(output_msg, "time");

            // 4. Traverse point cloud (core logic)
            size_t total_points = input_msg->height * input_msg->width;
            // Time conversion constants (适配整数型point_time)
            const uint32_t POINT_TIME_MAX = 2000000;  // 修正：改为uint32_t（整数）
            const uint32_t SEGMENT_SIZE = 200000;     // 修正：改为uint32_t（整数）

            if (debug_print_) {
                ROS_INFO("\n=== Processing Frame (Total Points: %zu) ===", total_points);
            }

            for (size_t i = 0; i < total_points; 
                 ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity,
                 ++iter_laserid, ++iter_point_time,
                 ++iter_x_out, ++iter_y_out, ++iter_z_out, ++iter_intensity_out,
                 ++iter_ring_out, ++iter_time_out)
            {
                // Copy basic data (x/y/z/intensity)
                *iter_x_out = *iter_x;
                *iter_y_out = *iter_y;
                *iter_z_out = *iter_z;
                *iter_intensity_out = *iter_intensity;

                // Core: laserid directly assigned to ring (no fallback)
                uint32_t raw_laserid = *iter_laserid;
                *iter_ring_out = static_cast<uint16_t>(raw_laserid);

                // ========== 修正：point_time整数类型转换逻辑 ==========
                uint32_t raw_point_time = *iter_point_time; // 直接读取整数型point_time
                float time = 0.0f;

                // 若raw_point_time有效（非0），按原逻辑转换；若为0，生成模拟时间
                if (raw_point_time > 0) {
                    // 手动范围限制（整数版）
                    uint32_t point_time_val = raw_point_time;
                    if (point_time_val > POINT_TIME_MAX) {
                        point_time_val = POINT_TIME_MAX;
                    }
                    // 取模+归一化（转换为0~0.1范围的浮点数）
                    float segment_remainder = static_cast<float>(point_time_val % SEGMENT_SIZE);
                    time = segment_remainder / static_cast<float>(POINT_TIME_MAX);
                } else {
                    // 模拟逐点时间：0~0.1s均匀分布（适配10Hz雷达）
                    time = static_cast<float>(i) / total_points * 0.1f;
                }
                *iter_time_out = time;

                // Debug print (English only, 修正raw_point_time显示格式为整数)
                if (debug_print_ && i < static_cast<size_t>(debug_print_count_)) {
                    ROS_INFO(
                        "Point [%zu] | x=%.3f, y=%.3f, z=%.3f, intensity=%.1f | laserid=%u : ring=%u | point_time=%u : time=%.6f",
                        i, *iter_x, *iter_y, *iter_z, *iter_intensity, raw_laserid, *iter_ring_out, raw_point_time, time
                    );
                }
            }

            // Publish converted point cloud
            publisher_.publish(output_msg);

            if (debug_print_) {
                ROS_INFO("=== Frame Processing Completed ===\n");
            }

        } catch (const std::exception& e) {
            ROS_ERROR("Error processing point cloud: %s", e.what());
        }
    }

    // Member variables
    ros::NodeHandle nh_;
    ros::Subscriber subscription_;
    ros::Publisher publisher_;
    std::string input_topic_;
    std::string output_topic_;
    int num_rings_;
    bool debug_print_;
    int debug_print_count_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "qs882velodyne_node	");
    ros::NodeHandle nh("~"); // Private namespace for correct parameter reading
    Qs882Velodyne_Node converter_node(nh);
    ros::spin();
    return 0;
}

