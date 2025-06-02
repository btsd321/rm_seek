#include "rm_utils/logger/log.hpp"
#include "seek_detector/seek_detector_option.h"

namespace fyt::seek
{
    static constexpr int INPUT_W = 640;      // Width of input
    static constexpr int INPUT_H = 640;      // Height of input
    static constexpr int NUM_CLASSES = 1;    // Number of classes
    static constexpr int NUM_ANCHORS = 8400; // Number of infer results
    static constexpr float DEFAULT_TOP_K = 10;
    static constexpr float MERGE_CONF_ERROR = 0.8;
    static constexpr float MERGE_MIN_IOU = 0.5;
    static constexpr const char* DEFAULT_MODEL_PATH = "/home/lixinhao/Project/rm_seek/seek_detector/data/model/best_openvino_model/best.xml";
    static constexpr const char* DEFAULT_DEVICE_NAME = "CPU";
    static constexpr bool DEFAULT_NMS_SWITCH = false;

    SeekDetectorOption::SeekDetectorOption()
    {
        this->input_width_ = INPUT_W;
        this->input_height_ = INPUT_H;
        this->class_num_ = NUM_CLASSES;
        this->confidence_threshold_ = MERGE_CONF_ERROR;
        this->model_path_ = std::filesystem::path(DEFAULT_MODEL_PATH);
        this->device_name_ = DEFAULT_DEVICE_NAME;
        this->confidence_threshold_ = MERGE_CONF_ERROR;
        this->top_k_ = DEFAULT_TOP_K;
        this->nms_threshold_ = MERGE_MIN_IOU;
        this->nms_switch_ = DEFAULT_NMS_SWITCH;
        this->anchor_num_ = NUM_ANCHORS;
    }

    SeekDetectorOption& SeekDetectorOption::setModelPath(const std::filesystem::path &model_path)
    {
        this->model_path_ = model_path;
        return *this;
    }

    SeekDetectorOption& SeekDetectorOption::setDeviceName(const std::string &device_name)
    {
        this->device_name_ = device_name;
        return *this;
    }

    SeekDetectorOption& SeekDetectorOption::setTopK(int top_k)
    {
        this->top_k_ = top_k;
        return *this;
    }

    SeekDetectorOption& SeekDetectorOption::setNmsThreshold(float nms_threshold)
    {
        this->nms_threshold_ = nms_threshold;
        return *this;
    }

    SeekDetectorOption& SeekDetectorOption::setNmsSwitch(bool nms_switch)
    {
        if(nms_switch) 
        {
            FYT_WARN("seek_detector", "暂不支持NMS，请设置NmsSwitch为false");
            throw std::runtime_error("暂不支持NMS，请设置NmsSwitch为false");
            return *this;
        }
        this->nms_switch_ = nms_switch;
        return *this;
    }

    SeekDetectorOption& SeekDetectorOption::setInputWidth(int input_width)
    {
        this->input_width_ = input_width;
        return *this;
    }


    SeekDetectorOption& SeekDetectorOption::setInputHeight(int input_height)
    {
        this->input_height_ = input_height;
        return *this;
    }

    SeekDetectorOption& SeekDetectorOption::setClassNum(int class_num)
    {
        this->class_num_ = class_num;
        return *this;
    }

    SeekDetectorOption& SeekDetectorOption::setAnchorNum(int anchor_num)
    {
        this->anchor_num_ = anchor_num;
        return *this;
    }

    SeekDetectorOption& SeekDetectorOption::setConfidenceThreshold(float confidence_threshold)
    {
        this->confidence_threshold_ = confidence_threshold;
        return *this;
    }

    int SeekDetectorOption::getInputWidth() const
    {
        return this->input_width_;
    }

    int SeekDetectorOption::getInputHeight() const
    {
        return this->input_height_;
    }

    int SeekDetectorOption::getClassNum() const
    {
        return this->class_num_;
    }

    int SeekDetectorOption::getAnchorNum() const
    {
        return this->anchor_num_;
    }

    const std::filesystem::path SeekDetectorOption::getModelPath() const
    {
        return this->model_path_;
    }

    const std::string SeekDetectorOption::getDeviceName() const
    {
        return this->device_name_;
    }

    float SeekDetectorOption::getConfidenceThreshold() const
    {
        return this->confidence_threshold_;
    }

    int SeekDetectorOption::getTopK() const
    {
        return this->top_k_;
    }

    float SeekDetectorOption::getNmsThreshold() const
    {
        return this->nms_threshold_;
    }

    bool SeekDetectorOption::getNmsSwitch() const
    {
        return this->nms_switch_;
    }
    
}
