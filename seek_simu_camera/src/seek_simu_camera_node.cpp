#include "seek_simu_camera/seek_simu_camera_node.h"

namespace fyt::seek
{
    // 将 vector<double> 转换为字符串
    static std::string vectorToString(const std::vector<double>& vec)
    {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < vec.size(); ++i)
        {
            oss << vec[i];
            if (i != vec.size() - 1)
            {
                oss << ", ";
            }
        }
        oss << "]";
        return oss.str();
    }

    SeekSimuCameraNode::SeekSimuCameraNode(const rclcpp::NodeOptions &options)
        : Node("seek_simu_camera_node", options)
    {
        // 声明参数 // Declare parameters
        this->declare_parameter<std::string>("video_path", "/home/lixinhao/Videos/robot_master_test_video.avi");
        this->declare_parameter<std::string>("img_topic_name", "simu_camera/image_raw");
        this->declare_parameter<std::string>("camera_info_topic_name", "simu_camera/camera_info");
        this->declare_parameter<int>("publish_rate", 30);
        this->declare_parameter("camera_info.width", 1280);
        this->declare_parameter("camera_info.height", 1024);
        this->declare_parameter("camera_info.distortion_model", "plumb_bob");
        this->declare_parameter("camera_info.d", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});
        this->declare_parameter("camera_info.k", std::vector<double>{525.0, 0.0, 320.0, 0.0, 525.0, 240.0, 0.0, 0.0, 1.0});
        this->declare_parameter("camera_info.r", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
        this->declare_parameter("camera_info.p", std::vector<double>{525.0, 0.0, 320.0, 0.0, 0.0, 525.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0});

        // 获取参数 // Get parameters
        this->get_parameter("video_path", video_path_);
        RCLCPP_INFO(this->get_logger(), "视频文件路径: %s", video_path_.c_str()); // Video file path
        this->get_parameter("img_topic_name", img_topic_name_);
        RCLCPP_INFO(this->get_logger(), "发布图像的主题名称: %s", img_topic_name_.c_str()); // Topic name for publishing images
        this->get_parameter("camera_info_topic_name", camera_info_topic_name_);
        RCLCPP_INFO(this->get_logger(), "发布相机信息的主题名称: %s", camera_info_topic_name_.c_str()); // Topic name for publishing camera info
        this->get_parameter("publish_rate", publish_rate_);
        RCLCPP_INFO(this->get_logger(), "发布频率: %d", publish_rate_); // Publishing rate
        this->get_parameter("camera_info.width", camera_info_.width);
        RCLCPP_INFO(this->get_logger(), "相机图像宽度: %d", camera_info_.width); // Camera image width
        this->get_parameter("camera_info.height", camera_info_.height);
        RCLCPP_INFO(this->get_logger(), "相机图像高度: %d", camera_info_.height); // Camera image height
        this->get_parameter("camera_info.distortion_model", camera_info_.distortion_model);
        RCLCPP_INFO(this->get_logger(), "畸变模型: %s", camera_info_.distortion_model.c_str()); // Distortion model
        // 获取参数并打印
        std::vector<double> camera_info_d;
        std::vector<double> camera_info_k;
        std::vector<double> camera_info_r;
        std::vector<double> camera_info_p;
        this->get_parameter("camera_info.d", camera_info_d);
        RCLCPP_INFO(this->get_logger(), "畸变参数: %s", vectorToString(camera_info_d).c_str()); // Distortion parameters
        this->get_parameter("camera_info.k", camera_info_k);
        RCLCPP_INFO(this->get_logger(), "内参矩阵: %s", vectorToString(camera_info_k).c_str()); // Intrinsic matrix
        this->get_parameter("camera_info.r", camera_info_r);
        RCLCPP_INFO(this->get_logger(), "旋转矩阵: %s", vectorToString(camera_info_r).c_str()); // Rotation matrix
        this->get_parameter("camera_info.p", camera_info_p);
        RCLCPP_INFO(this->get_logger(), "投影矩阵: %s", vectorToString(camera_info_p).c_str()); // Projection matrix
        //将参数传入camera_info_
        camera_info_.d = camera_info_d;
        for (size_t i = 0; i < camera_info_k.size() && i < camera_info_.k.size(); ++i)
        {
            camera_info_.k[i] = camera_info_k[i];
        }
        for(size_t i = 0; i < camera_info_r.size() && i < camera_info_.r.size(); ++i)
        {
            camera_info_.r[i] = camera_info_r[i];
        }
        for(size_t i = 0; i < camera_info_p.size() && i < camera_info_.p.size(); ++i)
        {
            camera_info_.p[i] = camera_info_p[i];
        }
        cap_.open(video_path_);
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "无法打开视频文件: %s", video_path_.c_str()); // Unable to open video file
            while(1);
            rclcpp::shutdown();
            return;
        }

        // 获取视频的原始帧率 // Get the original frame rate of the video
        raw_fps_ = cap_.get(cv::CAP_PROP_FPS);
        RCLCPP_INFO(this->get_logger(), "视频原始帧率: %d", raw_fps_); // Original frame rate of the video

        if(raw_fps_ % publish_rate_ != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "视频原始帧率必须是发布频率的整数倍"); // The publishing rate must be an integer multiple of the original frame rate of the video
            rclcpp::shutdown();
            return;
        }

        // 创建发布器 // Create publisher
        auto qos = rclcpp::SensorDataQoS();
        qos.keep_last(1);
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(img_topic_name_, qos);
        camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(img_topic_name_ + "/camera_info", 10);
        // 创建定时器 // Create timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / publish_rate_)),
            std::bind(&SeekSimuCameraNode::publishFrame, this));
        // 发布 CameraInfo
        camera_info_.header.stamp = this->now();
        camera_info_.header.frame_id = "camera_link";
        camera_info_publisher_->publish(camera_info_);
        RCLCPP_INFO(this->get_logger(), "发布相机信息: %s", camera_info_topic_name_.c_str()); // Publish camera info

        video_info_srv_ = this->create_service<rm_interfaces::srv::GetVideoInfo>(
            "get_video_info",
            std::bind(&SeekSimuCameraNode::getVideoInfoCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

    void SeekSimuCameraNode::getVideoInfoCallback(const std::shared_ptr<rm_interfaces::srv::GetVideoInfo::Request> request,
                                                  std::shared_ptr<rm_interfaces::srv::GetVideoInfo::Response> response)
    {
        // 获取视频信息并返回 // Get video info and return
        response->img_topic_name = img_topic_name_;
        response->publish_rate = this->publish_rate_;
        response->img_width = camera_info_.width;
        response->img_height = camera_info_.height;
    }

    void SeekSimuCameraNode::publishFrame()
    {
        static int frame_skip_count = 0; // 用于记录需要跳过的帧数
        // static int frame_counter = 0;   // 当前帧计数器

        // 计算需要跳过的帧数
        int skip_frames = static_cast<int>(raw_fps_ / publish_rate_) - 1;

        cv::Mat frame;
        while (frame_skip_count < skip_frames)
        {
            if (!cap_.read(frame))
            {
                RCLCPP_WARN(this->get_logger(), "视频播放结束，重新开始播放"); // 视频播放结束，重新开始
                cap_.set(cv::CAP_PROP_POS_FRAMES, 0); // 重置视频帧位置
                if (!cap_.read(frame))
                {
                    RCLCPP_ERROR(this->get_logger(), "无法重新读取视频文件: %s", video_path_.c_str()); // 无法重新读取视频
                    rclcpp::shutdown();
                    return;
                }
            }
            frame_skip_count++;
        }

        // 读取当前帧
        if (!cap_.read(frame))
        {
            RCLCPP_WARN(this->get_logger(), "视频播放结束，重新开始播放"); // 视频播放结束，重新开始
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0); // 重置视频帧位置
            if (!cap_.read(frame))
            {
                RCLCPP_ERROR(this->get_logger(), "无法重新读取视频文件: %s", video_path_.c_str()); // 无法重新读取视频
                rclcpp::shutdown();
                return;
            }
        }

        // 重置跳帧计数器
        frame_skip_count = 0;

        // 转换为 ROS 图像消息
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        image_publisher_->publish(*msg);
    }
}