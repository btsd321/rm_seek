#ifndef SEEK_DETECTOR_H
#define SEEK_DETECTOR_H

#include <filesystem>
#include <functional>
#include <future>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Dense>

#include <openvino/openvino.hpp>

#include "seek_detector/seek_object.h"
#include "seek_detector/seek_detector_option.h"


namespace fyt::seek
{
    class SeekDetector
    {
    public:
        using NmsCallbackType = std::function<void(std::vector<SeekObject> &, int64_t, const cv::Mat &)>;
        using CallbackType = std::function<void(SeekObject& , int64_t, const cv::Mat &)>;

    public:
        // Construct a new OpenVINO Detector object
        explicit SeekDetector(SeekDetectorOption& option);

        // Push an inference request to the detector
        std::future<bool> pushInput(const cv::Mat &rgb_img, int64_t timestamp_nanosec);

        //void setNmsCallback(NmsCallbackType callback);
        void setCallback(CallbackType callback);

        SeekDetectorOption& getOption();

    private:
        void init();
        // Do inference and call the infer_callback_ after inference
        bool processCallback(const cv::Mat resized_img,
                             Eigen::Matrix3f transform_matrix,
                             int64_t timestamp_nanosec,
                             const cv::Mat &src_img);

    private:
               
        SeekDetectorOption option_;
        std::mutex mtx_;

        NmsCallbackType infer_nms_callback_;
        CallbackType infer_callback_;

        std::unique_ptr<ov::Core> ov_core_;
        std::unique_ptr<ov::CompiledModel> compiled_model_;

        
    };
}

#endif // SEEK_DETECTOR_H