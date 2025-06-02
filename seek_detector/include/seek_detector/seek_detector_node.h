#ifndef SEEK_DETECTOR_NODE_H
#define SEEK_DETECTOR_NODE_H

#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "rm_interfaces/msg/seek_target.hpp"
#include "rm_interfaces/srv/ctrl_seek_detect_srv.hpp"
#include "seek_detector/seek_detector.h"
#include "seek_detector/seek_pose_estimator.h"
#include "rm_utils/heartbeat.hpp"
#include "rm_interfaces/srv/get_video_info.hpp"

namespace fyt::seek
{
    #ifndef MYDEBUG
    #define MYDEBUG
    #endif 
    class SeekDetectorNode : public rclcpp::Node
    {
    public:
        SeekDetectorNode(const rclcpp::NodeOptions &options);
   
    private:
        bool init_params();
        std::shared_ptr<SeekDetector> initDetector();

        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
        void inferResultCallback(SeekObject& seek_object,
                                 int64_t timestamp_nanosec,
                                 const cv::Mat &img);

        std::array<cv::Point2f, TARGET_POINT_NUM> points_match(std::array<cv::Point2f, TARGET_POINT_NUM> &last_input_points,
                                                std::array<cv::Point2f, TARGET_POINT_NUM> &current_input_points);
        std::array<cv::Point2f, TARGET_POINT_NUM> points_sort(std::array<cv::Point2f, TARGET_POINT_NUM> &current_input_points);

        // void createDebugPublishers();
        // void destroyDebugPublishers();

        // void setModeCallback(const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
        //                      std::shared_ptr<rm_interfaces::srv::SetMode::Response> response);
        // // Dynamic Parameter
        // rcl_interfaces::msg::SetParametersResult onSetParameters(
        //     std::vector<rclcpp::Parameter> parameters);
        // rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

        // Heartbeat
        HeartBeatPublisher::SharedPtr heartbeat_;

        // Image subscription
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

        // Target publisher
        std::string frame_id_;
        rclcpp::Publisher<rm_interfaces::msg::SeekTarget>::SharedPtr seek_pub_;

        // Enable/Disable Seek Detector
        rclcpp::Service<rm_interfaces::srv::CtrlSeekDetectSrv>::SharedPtr set_seek_mode_srv_;

        // Seek detector
        int requests_limit_;
        std::string img_topic_name_;
        int img_width_;
        int img_height_;
        std::queue<std::future<bool>> detect_requests_;
        std::shared_ptr<SeekDetector> seek_detector_;
        std::shared_ptr<SeekPoseEstimator> pose_estimator_;
        bool first_input_flag = true; // 第一次输入标志
        std::array<cv::Point2f, 4> last_input_points_; // 上一次输入的四个点

        Eigen::Matrix4d Q_;// 卡尔曼滤波器的过程噪声协方差矩阵
        Eigen::Matrix2d R_;// 卡尔曼滤波器的测量噪声协方差矩阵
        double dt_;// 时间间隔

        // 创建请求视频信息的服务客户端
        rclcpp::Client<rm_interfaces::srv::GetVideoInfo>::SharedPtr video_info_client_;
        
        // 可视化开关(大于0时开启可视化) // Enable visualization (greater than 0 to enable visualization)
        int enable_visualization_;
        // 结果可视化时的视频发布话题名称 // Topic name for result visualization video
        std::string result_img_topic_name_;
        // 结果可视化时的视频发布器 // Publisher
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_img_publisher_;

#ifdef MYDEBUG
        std::deque<SeekObject> seek_objects_;//创建缓存区，保存检测到的目标信息
#endif

        // // Seek params
        // EnemyColor detect_color_;
        // bool is_seek_;
        // bool is_big_seek_;

        // // For R tag detection
        // bool detect_r_tag_;
        // int binary_thresh_;

        // // Debug infomation
        // bool debug_;
        // image_transport::Publisher result_img_pub_;
    };
}

#endif // SEEK_DETECTOR_NODE_H