
#ifndef RM_UTILS_HEARTBEAT_HPP_
#define RM_UTILS_HEARTBEAT_HPP_

// std
#include <memory>
#include <thread>
// ros2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>
#include "rm_utils/heartbeat.hpp"

namespace fyt
{
    class HeartBeatPublisher
    {
    public:
        using SharedPtr = std::shared_ptr<HeartBeatPublisher>;

        static SharedPtr create(rclcpp::Node *node);

        ~HeartBeatPublisher();

    private:
        explicit HeartBeatPublisher(rclcpp::Node *node);

        std_msgs::msg::Int64 message_;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
        std::thread pub_thread_;

        // Heartbeat
        HeartBeatPublisher::SharedPtr heartbeat_;
    };
} // namespace fyt

#endif // RM_UTILS_HEARTBEAT_HPP_
