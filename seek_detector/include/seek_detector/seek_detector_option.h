#ifndef SEEK_DETECTOR_OPTION_H
#define SEEK_DETECTOR_OPTION_H

#include <filesystem>
#include <functional>
#include <future>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Dense>

#include <openvino/openvino.hpp>

#include "seek_detector/seek_object.h"

namespace fyt::seek
{
    class SeekDetectorOption
    {
    private:
        int input_width_;
        int input_height_;
        int class_num_;
        int anchor_num_;
        float confidence_threshold_;
        std::filesystem::path model_path_;
        std::string device_name_;
        int top_k_;
        float nms_threshold_;
        bool nms_switch_;// true时使用NMS，false时不使用NMS，只输出最大的置信度的目标
    public:
        SeekDetectorOption();

        //setters
        SeekDetectorOption& setModelPath(const std::filesystem::path &model_path);
        SeekDetectorOption& setDeviceName(const std::string &device_name);
        SeekDetectorOption& setTopK(int top_k);
        SeekDetectorOption& setNmsThreshold(float nms_threshold);
        SeekDetectorOption& setNmsSwitch(bool nms_switch);
        SeekDetectorOption& setInputWidth(int input_width);
        SeekDetectorOption& setInputHeight(int input_height);
        SeekDetectorOption& setClassNum(int class_num);
        SeekDetectorOption& setAnchorNum(int anchor_num);
        SeekDetectorOption& setConfidenceThreshold(float confidence_threshold);

        //getters
        int getInputWidth() const;
        int getInputHeight() const;
        int getClassNum() const;
        int getAnchorNum() const;
        const std::filesystem::path getModelPath() const;
        const std::string getDeviceName() const;
        float getConfidenceThreshold() const;
        int getTopK() const;
        float getNmsThreshold() const;
        bool getNmsSwitch() const;

    };
}

#endif // SEEK_DETECTOR_H