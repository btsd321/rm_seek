#include <rclcpp_components/register_node_macro.hpp>
#include <cv_bridge/cv_bridge.h>
#include <seek_detector/seek_detector_node.h>
#include <seek_detector/seek_detector_option.h>
#include "rm_utils/logger/log.hpp"


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

    SeekDetectorNode::SeekDetectorNode(const rclcpp::NodeOptions &options) : Node("seek_detector_node", options)
    {
        FYT_REGISTER_LOGGER("seek_detector", "~/fyt-log", INFO);
        FYT_INFO("seek_detector", "Starting SeekDetectorNode!");

        if(!init_params())
        {
            RCLCPP_ERROR(get_logger(), "Failed to initialize parameters!");
            return;
        }

        try
        {
            this->pose_estimator_ = std::make_shared<SeekPoseEstimator>(this->Q_, this->R_, this->dt_);
                    RCLCPP_INFO(this->get_logger(), "Initialized pose_estimator_!");
            RCLCPP_INFO(this->get_logger(), "Initialized Kalman filter!");
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize pose_estimator_! Error: %s", e.what());
            return;
        }
        
        

        // Detector
        try
        {
            seek_detector_ = initDetector();
            RCLCPP_INFO(get_logger(), "Initialized detector!");

            // Seek Publisher
            seek_pub_ = this->create_publisher<rm_interfaces::msg::SeekTarget>("seek_detector/seek_target",
                                                                            rclcpp::SensorDataQoS());

            auto qos = rclcpp::SensorDataQoS();
            qos.keep_last(1);
            // Image Subscriber
            img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                img_topic_name_, qos, std::bind(&SeekDetectorNode::imageCallback, this, std::placeholders::_1));
            RCLCPP_INFO(get_logger(), "Initialized image subscriber!");
            // // Set mode service
            // set_seek_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
            //     "seek_detector/set_mode",
            //     std::bind(
            //         &SeekDetectorNode::setModeCallback, this, std::placeholders::_1, std::placeholders::_2));

            // Heartbeat
            heartbeat_ = HeartBeatPublisher::create(this);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            RCLCPP_ERROR(get_logger(), "SeekDetectorNode init failed!");
            return;
        }

        RCLCPP_INFO(get_logger(), "SeekDetectorNode started!");
    }

    bool SeekDetectorNode::init_params()
    {
        this->declare_parameter<std::string>("frame_id", "camera_link");
        this->declare_parameter<int>("requests_limit", 5);
        this->declare_parameter<std::vector<double>>("ekf_filter.process_noise_cov", std::vector<double>(16, 0.0));
        this->declare_parameter<std::vector<double>>("ekf_filter.measurement_noise_cov", std::vector<double>(4, 0.0));
        this->declare_parameter<int>("enable_visualization", 1);
        this->declare_parameter<std::string>("result_img_topic_name", "seek_detector/result_image");

        std::vector<double> process_noise_cov;
        std::vector<double> measurement_noise_cov;
        // 获取参数
        this->get_parameter("frame_id", frame_id_);
        RCLCPP_INFO(get_logger(), "get parameter frame_id: %s", frame_id_.c_str());
        this->get_parameter("requests_limit", requests_limit_);
        RCLCPP_INFO(get_logger(), "get parameter requests_limit: %d", requests_limit_);  
        this->get_parameter("ekf_filter.process_noise_cov", process_noise_cov);
        RCLCPP_INFO(get_logger(), "get parameter ekf_filter.process_noise_cov: %s", vectorToString(process_noise_cov).c_str());
        this->get_parameter("ekf_filter.measurement_noise_cov", measurement_noise_cov);
        RCLCPP_INFO(get_logger(), "get parameter ekf_filter.measurement_noise_cov: %s", vectorToString(measurement_noise_cov).c_str());
        this->get_parameter("enable_visualization", enable_visualization_);
        RCLCPP_INFO(get_logger(), "get parameter enable_visualization: %d", enable_visualization_);
        this->get_parameter("result_img_topic_name", result_img_topic_name_);
        RCLCPP_INFO(get_logger(), "get parameter result_img_topic_name: %s", result_img_topic_name_.c_str());
        if(enable_visualization_)
        {
            result_img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(result_img_topic_name_, 1);
        }
        else
        {
            result_img_publisher_ = nullptr;
        }
        
        if (process_noise_cov.size() == 16)
        {
            for (size_t i = 0; i < 4; ++i)
            {
                for (size_t j = 0; j < 4; ++j)
                {
                    this->Q_(i, j) = process_noise_cov[i * 4 + j];
                }
            }
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Invalid process_noise_cov size. Using default values.");
            this->Q_ = Eigen::Matrix4d::Identity() * 0.01; // 默认值
        }

        if (measurement_noise_cov.size() == 4)
        {
            for (size_t i = 0; i < 2; ++i)
            {
                for (size_t j = 0; j < 2; ++j)
                {
                    this->R_(i, j) = measurement_noise_cov[i * 2 + j];
                }
            }
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Invalid measurement_noise_cov size. Using default values.");
            this->R_ = Eigen::Matrix2d::Identity() * 0.1; // 默认值
        }

        video_info_client_ = this->create_client<rm_interfaces::srv::GetVideoInfo>("get_video_info");
        while (!video_info_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }
        auto request = std::make_shared<rm_interfaces::srv::GetVideoInfo::Request>();
        auto future_result = video_info_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future_result.get(); // 获取服务响应
            RCLCPP_INFO(get_logger(), "Get video info success!");
            double& publish_rate = response->publish_rate; // 用于存储帧率
            RCLCPP_INFO(get_logger(), "get parameter publish_rate: %f", publish_rate);
            img_topic_name_ = response->img_topic_name; // 用于存储图像话题名称
            RCLCPP_INFO(get_logger(), "get parameter img_topic_name: %s", img_topic_name_.c_str());
            this->img_width_ = response->img_width; // 用于存储图像宽度
            RCLCPP_INFO(get_logger(), "get parameter img_width: %d", img_width_);
            this->img_height_ = response->img_height; // 用于存储图像高度
            RCLCPP_INFO(get_logger(), "get parameter img_height: %d", img_height_);

            // 检查 publish_rate 是否有效
            if (publish_rate > 0.0)
            {
                dt_ = 1.0 / publish_rate; // 计算 dt
                RCLCPP_INFO(this->get_logger(), "Calculated dt: %f", dt_);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Invalid publish_rate value. Using default dt = 0.1");
                dt_ = 0.1; // 默认值
            }
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to call service get_video_info");
            return false;
        }
        return true;
    }

    std::shared_ptr<SeekDetector> SeekDetectorNode::initDetector()
    {
        SeekDetectorOption option;
        // 可以添加参数设置，不添加则使用默认设置
        seek_detector_ = std::make_shared<SeekDetector>(option);
        seek_detector_->setCallback(std::bind(&SeekDetectorNode::inferResultCallback,
                                              this,
                                              std::placeholders::_1,
                                              std::placeholders::_2,
                                              std::placeholders::_3));
        return seek_detector_;
    }

    std::array<cv::Point2f, TARGET_POINT_NUM> SeekDetectorNode::points_sort(std::array<cv::Point2f, TARGET_POINT_NUM>& current_input_points)
    {
        // 对输入的点进行排序分为左上、右上、左下、右下
        if(TARGET_POINT_NUM != 4)
        {
            throw std::runtime_error("TARGET_POINT_NUM must be 4");
            return current_input_points;
        }

        std::array<cv::Point2f, TARGET_POINT_NUM> sorted_points;

        // 计算中心点
        cv::Point2f center_point(0, 0);
        for(size_t i = 0; i < TARGET_POINT_NUM; ++i)
        {
            center_point.x += current_input_points[i].x;
            center_point.y += current_input_points[i].y;
        }
        center_point.x /= TARGET_POINT_NUM;
        center_point.y /= TARGET_POINT_NUM;

        std::array<Eigen::Vector2d, TARGET_POINT_NUM> points_vectors;
        for(auto it = current_input_points.begin(); it!= current_input_points.end(); ++it)
        {
            points_vectors[it - current_input_points.begin()] = Eigen::Vector2d(it->x - center_point.x, it->y - center_point.y);
        }

        // 以centert_point为原点，计算x轴负方向转到向量方向的夹角大小对输入点进行排序，结果存入sorted_points
        std::multimap<double, cv::Point2f> sort_angle_map;
        for(size_t i = 0; i < TARGET_POINT_NUM; ++i)
        {
            double angle;
            if(points_vectors[i].x() == 0 && points_vectors[i].y() == 0)
            {
                angle = 0;
            }
            else if(points_vectors[i].x() == 0)
            {
                angle = points_vectors[i].y() > 0 ? (3* M_PI / 2) : M_PI / 2;
            }
            else
            {
                angle = M_PI + std::atan2(points_vectors[i].y(), points_vectors[i].x());
            }

            sort_angle_map.insert(std::make_pair(angle, current_input_points[i]));
        }
        // 取出排序后的点
        auto it = sort_angle_map.begin();
        for(size_t i = 0; i < TARGET_POINT_NUM; ++i)
        {
            sorted_points[i] = it->second;
            ++it;
        }
        return sorted_points;
    }

    std::array<cv::Point2f, TARGET_POINT_NUM> SeekDetectorNode::points_match(std::array<cv::Point2f, TARGET_POINT_NUM> &last_input_points,
        std::array<cv::Point2f, TARGET_POINT_NUM> &current_input_points)
    {
        std::array<cv::Point2f, TARGET_POINT_NUM> matched_points;
        if(TARGET_POINT_NUM < 3)
        {
            matched_points = current_input_points;
            throw std::runtime_error("TARGET_POINT_NUM must be greater than or equal to 3");
            return matched_points;
        }

        // 匹配关系有三种：1：无需改变正好匹配；2：需要顺时针旋转匹配；3：需要逆时针旋转匹配
        auto cal_distance = [](cv::Point2f& p1, cv::Point2f& p2) -> double
        {
            return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
        };

        double distance21 = cal_distance(last_input_points[0], current_input_points[1]);
        double distance23 = cal_distance(last_input_points[2], current_input_points[1]);
        double distance22 = cal_distance(last_input_points[1], current_input_points[1]);

        if(distance22 < distance21 && distance22 < distance23)
        {
            // 无需改变正好匹配
            matched_points = current_input_points;
            return matched_points;
        }
        
        if(distance23 < distance22)
        {
            // 顺时针旋转匹配
            for(size_t i = 0; i < TARGET_POINT_NUM; ++i)
            {
                matched_points[i] = current_input_points[(i + 1) % TARGET_POINT_NUM];
            }
            RCLCPP_INFO(this->get_logger(), "顺时针旋转匹配");
            return matched_points;
        }

        if(distance21 < distance22)
        {
            // 逆时针旋转匹配
            for(size_t i = 0; i < TARGET_POINT_NUM; ++i)
            {
                matched_points[i] = current_input_points[(i + TARGET_POINT_NUM - 1) % TARGET_POINT_NUM];
            }
            RCLCPP_INFO(this->get_logger(), "逆时针旋转匹配");
            return matched_points;
        }

        matched_points = current_input_points;
        return matched_points;
    }

    void SeekDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
    {
        // Limits request size
        while (detect_requests_.size() > static_cast<size_t>(requests_limit_))
        {
            detect_requests_.front().get();
            detect_requests_.pop();
        }

        auto timestamp = rclcpp::Time(img_msg->header.stamp);
        frame_id_ = img_msg->header.frame_id;
        auto img = cv_bridge::toCvCopy(img_msg, "rgb8")->image;

        // Push image to detector
        detect_requests_.push(seek_detector_->pushInput(img, timestamp.nanoseconds()));
    }

    void SeekDetectorNode::inferResultCallback(SeekObject &seek_object,
                                               int64_t timestamp_nanosec,
                                               const cv::Mat &img)
    {
        //打印滤波前的四个点坐标
        // RCLCPP_INFO(get_logger(), "Before filtering: taret_point0: %f, %f, target_point1: %f, %f, target_point2: %f, %f, target_point3: %f, %f",
        //                                             seek_object.get_point(0).x, seek_object.get_point(0).y,
        //                                             seek_object.get_point(1).x, seek_object.get_point(1).y,
        //                                             seek_object.get_point(2).x, seek_object.get_point(2).y,
        //                                             seek_object.get_point(3).x, seek_object.get_point(3).y);
        RCLCPP_INFO(get_logger(), "%f, %f, %f, %f, %f, %f, %f, %f",
                        seek_object.get_point(0).x, seek_object.get_point(0).y,
                        seek_object.get_point(1).x, seek_object.get_point(1).y,
                        seek_object.get_point(2).x, seek_object.get_point(2).y,
                        seek_object.get_point(3).x, seek_object.get_point(3).y);

        // 更新滤波器
        auto current_points = seek_object.get_points();
        // 排序
        auto sorted_points = this->points_sort(current_points);
        if(this->first_input_flag)
        {
            this->pose_estimator_->setFirstState(sorted_points);
            first_input_flag = false;
            this->last_input_points_ = seek_object.get_points();
        }
        else
        {   
            auto matched_points = points_match(last_input_points_, sorted_points);
            this->pose_estimator_->update(matched_points);
            this->last_input_points_ = matched_points;
        }
        

        // 获取滤波后的点
        auto filtered_points = pose_estimator_->getFilteredPoints();

        // 打印滤波后的四个点坐标
        // RCLCPP_INFO(get_logger(), "After filtering: taret_point0: %f, %f, target_point1: %f, %f, target_point2: %f, %f, target_point3: %f, %f",
        //                                             filtered_points[0].x, filtered_points[0].y,
        //                                             filtered_points[1].x, filtered_points[1].y,
        //                                             filtered_points[2].x, filtered_points[2].y,
        //                                             filtered_points[3].x, filtered_points[3].y);

        // Init seek target msg
        rm_interfaces::msg::SeekTarget seek_msg;
        seek_msg.header.frame_id = frame_id_;
        seek_msg.header.stamp = rclcpp::Time(timestamp_nanosec);
        seek_msg.box_center.x = seek_object.get_box().get_center().x;
        seek_msg.box_center.y = seek_object.get_box().get_center().y;
        seek_msg.box_width = seek_object.get_box().get_f32_width();
        seek_msg.box_height = seek_object.get_box().get_f32_height();
        seek_msg.confidence = seek_object.get_score();
        for (size_t i = 0; i < TARGET_POINT_NUM; ++i)
        {
            seek_msg.target_points[i].x = filtered_points[i].x;
            seek_msg.target_points[i].y = filtered_points[i].y;
        }

        // 可视化
        if(result_img_publisher_)
        {
            // 获取四个点
            cv::Mat result_img;
            cv::cvtColor(img, result_img, cv::COLOR_RGB2BGR);

            cv::Point p1(filtered_points[0].x, filtered_points[0].y);
            cv::Point p2(filtered_points[1].x, filtered_points[1].y);
            cv::Point p3(filtered_points[2].x, filtered_points[2].y);
            cv::Point p4(filtered_points[3].x, filtered_points[3].y);
            
            // 画出四边形框
            // cv::line(result_img, p1, p2, cv::Scalar(0, 255, 0), 2); // 绿色线,滤波后的四个点
            // cv::line(result_img, p2, p3, cv::Scalar(0, 255, 0), 2);
            // cv::line(result_img, p3, p4, cv::Scalar(0, 255, 0), 2);
            // cv::line(result_img, p4, p1, cv::Scalar(0, 255, 0), 2);

            cv::line(result_img, sorted_points[0], sorted_points[1], cv::Scalar(0, 0, 255), 2); // 红色线,滤波前的四个点
            cv::line(result_img, sorted_points[1], sorted_points[2], cv::Scalar(0, 0, 255), 2);
            cv::line(result_img, sorted_points[2], sorted_points[3], cv::Scalar(0, 0, 255), 2);
            cv::line(result_img, sorted_points[3], sorted_points[0], cv::Scalar(0, 0, 255), 2);

            // 将结果图像发布
            auto result_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result_img).toImageMsg();
            result_img_publisher_->publish(*result_msg);
        }

        // Publish seek target msg
        seek_pub_->publish(seek_msg);
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(fyt::seek::SeekDetectorNode)