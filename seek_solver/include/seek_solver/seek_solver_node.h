#ifndef SEEK_SOLVER_NODE_H
#define SEEK_SOLVER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <rm_utils/math/pnp_solver.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace fyt::seek
{
    using namespace std::chrono_literals;

    class SeekSolverNode : public rclcpp::Node
    {
    public:
        SeekSolverNode(const rclcpp::NodeOptions &options);

    private:
        bool init_params();
        bool init_solver();
        void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

        std::unique_ptr<PnPSolver> solver_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        sensor_msgs::msg::CameraInfo camera_info_; // 相机参数 // Camera parameters
        rclcpp::TimerBase::SharedPtr timer_;
        bool get_camera_info_flag_ = false; // 相机参数是否获取 // Whether the camera parameters have been obtained
    };

    SeekSolverNode::SeekSolverNode(const rclcpp::NodeOptions &options): Node("seek_solver_node", options)
    {
        init_params();

        timer_ = this->create_wall_timer(
            500ms, std::bind(&SeekSolverNode::init_solver, this));
    }

    bool SeekSolverNode::init_params()
    {
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "seek_simu_camera/camera_info", 10, std::bind(&SeekSolverNode::camera_info_callback, this, std::placeholders::_1));
        return true;
    }

    bool SeekSolverNode::init_solver()
    {
        static int init_count = 0;
        if (!get_camera_info_flag_)
        {
            init_count++;
            if(init_count > 10)
            {
                RCLCPP_ERROR(this->get_logger(), "Camera info not received, please check the camera");
                timer_->cancel();
            }
            return false;
        }

        // TODO: 初始化 PnP 解算器 // Initialize PnP solver

        
        return true;
    }

    void SeekSolverNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        this->camera_info_ = *msg;
        get_camera_info_flag_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera info received");
    }
} // namespace fyt::seek

#endif // SEEK_SOLVER_NODE_H