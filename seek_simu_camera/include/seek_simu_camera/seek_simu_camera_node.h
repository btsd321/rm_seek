#ifndef SEEK_SIMU_CAMERA_NODE_H
#define SEEK_SIMU_CAMERA_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "rm_interfaces/srv/get_video_info.hpp"
#include <sensor_msgs/msg/camera_info.hpp>

namespace fyt::seek
{
    class SeekSimuCameraNode : public rclcpp::Node
    {
    public:
        explicit SeekSimuCameraNode(const rclcpp::NodeOptions &options);

    private:
        void publishFrame(); // 发布视频帧的函数 // Function to publish video frames
        void getVideoInfoCallback(const std::shared_ptr<rm_interfaces::srv::GetVideoInfo::Request> request,
                                   std::shared_ptr<rm_interfaces::srv::GetVideoInfo::Response> response); // 获取视频信息的回调函数 // Callback function to get video info

        // 视频捕获对象 // Video capture object
        cv::VideoCapture cap_;

        // 发布器 // Publisher
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
        // 定时器 // Timer
        rclcpp::TimerBase::SharedPtr timer_;

        //视频原始帧率
        int raw_fps_;

        // 参数 // Parameters
        std::string video_path_; // 视频文件路径 // Path to the video file
        std::string img_topic_name_; // 发布的主题名称 // Name of the topic to publish
        std::string camera_info_topic_name_;// 发布的相机参数主题名称 // Name of the topic to publish camera info
        int publish_rate_;    // 发布频率（帧率） // Publishing rate (frame rate)
        sensor_msgs::msg::CameraInfo camera_info_; // 相机参数 // Camera parameters

        rclcpp::Service<rm_interfaces::srv::GetVideoInfo>::SharedPtr video_info_srv_; // 获取相机参数的服务 // Service to get camera info
    };
}

// 注册组件 // Register the component
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::seek::SeekSimuCameraNode)

#endif // SEEK_SIMU_CAMERA_NODE_H