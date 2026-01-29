/**
 * @file depth_to_pointcloud_node.cpp
 * @brief Convert depth image to PointCloud2 for Berxel P100R
 * 
 * Converts RGBD depth images to PointCloud2 messages
 * with configurable parameters for the P100R camera.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/opencv.hpp>
#include <memory>
#include <string>

class DepthToPointCloudNode : public rclcpp::Node
{
public:
    DepthToPointCloudNode() : Node("depth_to_pointcloud_node")
    {
        // Declare parameters
        this->declare_parameter("depth_topic", "/berxel/depth/image_raw");
        this->declare_parameter("camera_info_topic", "/berxel/depth/camera_info");
        this->declare_parameter("output_topic", "/berxel/depth/points");
        this->declare_parameter("output_frame", "berxel_depth_optical_frame");
        
        // Berxel P100R default intrinsics (can be overridden by camera_info)
        this->declare_parameter("fx", 460.0);
        this->declare_parameter("fy", 460.0);
        this->declare_parameter("cx", 320.0);
        this->declare_parameter("cy", 240.0);
        
        // Processing parameters
        this->declare_parameter("depth_scale", 0.001);  // mm to meters
        this->declare_parameter("min_depth", 0.3);  // meters
        this->declare_parameter("max_depth", 5.0);  // meters
        this->declare_parameter("decimation", 2);  // Downsample factor
        this->declare_parameter("use_camera_info", true);
        
        // Get parameters
        depth_topic_ = this->get_parameter("depth_topic").as_string();
        camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        output_frame_ = this->get_parameter("output_frame").as_string();
        
        fx_ = this->get_parameter("fx").as_double();
        fy_ = this->get_parameter("fy").as_double();
        cx_ = this->get_parameter("cx").as_double();
        cy_ = this->get_parameter("cy").as_double();
        
        depth_scale_ = this->get_parameter("depth_scale").as_double();
        min_depth_ = this->get_parameter("min_depth").as_double();
        max_depth_ = this->get_parameter("max_depth").as_double();
        decimation_ = this->get_parameter("decimation").as_int();
        use_camera_info_ = this->get_parameter("use_camera_info").as_bool();
        
        // Subscribers
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic_, rclcpp::SensorDataQoS(),
            std::bind(&DepthToPointCloudNode::depth_callback, this, std::placeholders::_1));
        
        if (use_camera_info_) {
            camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                camera_info_topic_, rclcpp::SensorDataQoS(),
                std::bind(&DepthToPointCloudNode::camera_info_callback, this, std::placeholders::_1));
        }
        
        // Publisher
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_, rclcpp::SensorDataQoS());
        
        RCLCPP_INFO(this->get_logger(), "Depth to PointCloud Node initialized");
        RCLCPP_INFO(this->get_logger(), "  Depth topic: %s", depth_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Decimation: %d", decimation_);
    }

private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        // Update intrinsics from camera_info
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        
        camera_info_received_ = true;
        
        RCLCPP_INFO_ONCE(this->get_logger(), 
            "Camera info received: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
            fx_, fy_, cx_, cy_);
    }
    
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (use_camera_info_ && !camera_info_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Waiting for camera_info...");
            return;
        }
        
        try {
            // Convert to OpenCV
            cv_bridge::CvImagePtr cv_ptr;
            
            if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Unsupported depth encoding: %s", 
                    msg->encoding.c_str());
                return;
            }
            
            // Convert to point cloud
            auto cloud_msg = convert_to_pointcloud(cv_ptr->image, msg->header, msg->encoding);
            
            if (cloud_msg) {
                pointcloud_pub_->publish(*cloud_msg);
            }
            
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    
    sensor_msgs::msg::PointCloud2::SharedPtr convert_to_pointcloud(
        const cv::Mat& depth_image,
        const std_msgs::msg::Header& header,
        const std::string& encoding)
    {
        int height = depth_image.rows / decimation_;
        int width = depth_image.cols / decimation_;
        
        // Create point cloud message
        auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud_msg->header = header;
        cloud_msg->header.frame_id = output_frame_;
        cloud_msg->height = 1;
        cloud_msg->is_dense = false;
        
        // Set up fields
        sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(width * height);
        
        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
        
        int valid_points = 0;
        
        for (int v = 0; v < depth_image.rows; v += decimation_) {
            for (int u = 0; u < depth_image.cols; u += decimation_) {
                float depth;
                
                if (encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                    uint16_t raw_depth = depth_image.at<uint16_t>(v, u);
                    depth = raw_depth * depth_scale_;
                } else {
                    depth = depth_image.at<float>(v, u);
                }
                
                // Check valid depth range
                if (depth < min_depth_ || depth > max_depth_ || !std::isfinite(depth)) {
                    *iter_x = std::numeric_limits<float>::quiet_NaN();
                    *iter_y = std::numeric_limits<float>::quiet_NaN();
                    *iter_z = std::numeric_limits<float>::quiet_NaN();
                } else {
                    // Deproject to 3D (optical frame: Z forward, X right, Y down)
                    *iter_x = (u - cx_) * depth / fx_;
                    *iter_y = (v - cy_) * depth / fy_;
                    *iter_z = depth;
                    valid_points++;
                }
                
                ++iter_x;
                ++iter_y;
                ++iter_z;
            }
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Generated point cloud with %d/%d valid points",
            valid_points, width * height);
        
        return cloud_msg;
    }

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    
    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    
    // Parameters
    std::string depth_topic_;
    std::string camera_info_topic_;
    std::string output_topic_;
    std::string output_frame_;
    
    double fx_, fy_, cx_, cy_;
    double depth_scale_;
    double min_depth_;
    double max_depth_;
    int decimation_;
    bool use_camera_info_;
    
    bool camera_info_received_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DepthToPointCloudNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
