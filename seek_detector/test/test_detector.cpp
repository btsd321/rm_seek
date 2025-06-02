#include <gtest/gtest.h>
// ros2
#include <opencv2/imgproc.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/utilities.hpp>
// std
#include <memory>
// opencv
#include <opencv2/opencv.hpp>
// project
#include "rm_utils/common.hpp"
#include "rm_utils/url_resolver.hpp"
#include "seek_detector/seek_detector.h"
#include "seek_detector/seek_detector_node.h"

using namespace fyt;
using namespace fyt::seek;

TEST(seek_detector, api_test)
{
    ASSERT_EQ(4, 2 + 2);
}

TEST(seek_detector, node_test)
{
    ASSERT_EQ(4, 2 + 2);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}