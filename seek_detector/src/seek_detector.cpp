// Copyright 2023 Yunlong Feng
//
// Additional modifications and features by Chengfu Zou, 2024.
//
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// std
#include <algorithm>
#include <numeric>
#include <unordered_map>
// third party
#include <opencv2/imgproc.hpp>
// project
#include "rm_utils/common.hpp"
#include "rm_utils/logger/log.hpp"
#include "seek_detector/seek_object.h"
#include "seek_detector/seek_detector.h"

#ifndef MYDEBUG
#define MYDEBUG 1
#endif

namespace fyt::seek
{
    static cv::Mat letterbox(SeekDetectorOption& option, const cv::Mat &img, Eigen::Matrix3f &transform_matrix)
    {

        // Get current image shape [height, width]
        std::vector<int> new_shape = {option.getInputWidth(), option.getInputHeight()};
        int img_h = img.rows;
        int img_w = img.cols;

        // Compute scale ratio(new / old) and target resized shape
        float scale =
            std::min(new_shape[1] * 1.0 / img_h, new_shape[0] * 1.0 / img_w);
        int resize_h = static_cast<int>(round(img_h * scale));
        int resize_w = static_cast<int>(round(img_w * scale));

        // Compute padding
        int pad_h = new_shape[1] - resize_h;
        int pad_w = new_shape[0] - resize_w;

        // Resize and pad image while meeting stride-multiple constraints
        cv::Mat resized_img;
        cv::resize(img, resized_img, cv::Size(resize_w, resize_h));

        // Divide padding into 2 sides
        float half_h = pad_h * 1.0 / 2;
        float half_w = pad_w * 1.0 / 2;

        // Compute padding boarder
        int top = static_cast<int>(round(half_h - 0.1));
        int bottom = static_cast<int>(round(half_h + 0.1));
        int left = static_cast<int>(round(half_w - 0.1));
        int right = static_cast<int>(round(half_w + 0.1));

        // Compute point transform_matrix
        transform_matrix << 1.0 / scale, 0, -half_w / scale,
                            0, 1.0 / scale, -half_h / scale,
                            0, 0, 1;

        // Add border
        cv::copyMakeBorder(resized_img, resized_img, top, bottom, left, right,
                           cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

        return resized_img;
    }

    // Decode output tensor
    static void generateUniqueProposal(SeekDetectorOption &option,
                                       SeekObject &output_obj, const cv::Mat &output_buffer,
                                       const Eigen::Matrix<float, 3, 3> &transform_matrix, int class_id)
    {
        // 遍历查找最大的置信度的目标放入到output_obj中
        float max_score = -1000.0;
        int out_anchor_idx = -1;
        for (int anchor_idx = 0; anchor_idx < option.getAnchorNum(); anchor_idx++)
        {
            cv::Mat num_scores =
                output_buffer.col(anchor_idx)
                    .rowRange(4, 4 + option.getClassNum());
            // 获取第class_id类的置信度
            float class_score = num_scores.at<float>(class_id);
            if (class_score > max_score)
            {
                max_score = class_score;

                out_anchor_idx = anchor_idx;
            }
        }
        float center_x = output_buffer.at<float>(0, out_anchor_idx);
        float center_y = output_buffer.at<float>(1, out_anchor_idx);
        float box_width = output_buffer.at<float>(2, out_anchor_idx);
        float box_height = output_buffer.at<float>(3, out_anchor_idx);
        cv::Point2f target1 = cv::Point2f(output_buffer.at<float>(4 + option.getClassNum(), out_anchor_idx), output_buffer.at<float>(5 + option.getClassNum(), out_anchor_idx));
        cv::Point2f target2 = cv::Point2f(output_buffer.at<float>(6 + option.getClassNum(), out_anchor_idx), output_buffer.at<float>(7 + option.getClassNum(), out_anchor_idx));
        cv::Point2f target3 = cv::Point2f(output_buffer.at<float>(8 + option.getClassNum(), out_anchor_idx), output_buffer.at<float>(9 + option.getClassNum(), out_anchor_idx));
        cv::Point2f target4 = cv::Point2f(output_buffer.at<float>(10 + option.getClassNum(), out_anchor_idx), output_buffer.at<float>(11 + option.getClassNum(), out_anchor_idx));

        BoxRect box(center_x, center_y, box_width, box_height);
        output_obj = SeekObject(box, max_score, target1, target2, target3, target4).transform(transform_matrix);
#if MYDEBUG
        static unsigned int count = 0;
        count++;
        if(count % 15 == 0)
        {
            std::cout << "center_x: " << center_x 
                << ", center_y: " << center_y
                << ", box_width: " << box_width
                << ", box_height: " << box_height
                << ", target1: " << target1
                << ", target2: " << target2
                << ", target3: " << target3
                << ", target4: " << target4
                << std::endl;
        }
#endif
    }

    SeekDetector::SeekDetector(SeekDetectorOption &option)
    {
        this->option_ = option;
        init();
    }

    void SeekDetector::init()
    {
        if (ov_core_ == nullptr)
        {
            ov_core_ = std::make_unique<ov::Core>();
        }

        auto model = ov_core_->read_model(this->option_.getModelPath());

        // Set infer type
        ov::preprocess::PrePostProcessor ppp(model);
        // Set input output precision
        auto elem_type = (this->option_.getDeviceName() == "GPU") ? ov::element::f16 : ov::element::f32;
        auto perf_mode =
        (this->option_.getDeviceName() == "GPU")
                ? ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT)
                : ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY);
        ppp.input().tensor().set_element_type(elem_type);
        ppp.output().tensor().set_element_type(elem_type);

        // Compile model
        compiled_model_ = std::make_unique<ov::CompiledModel>(
            ov_core_->compile_model(model, this->option_.getDeviceName(), perf_mode));
    }

    std::future<bool> SeekDetector::pushInput(const cv::Mat &rgb_img,
                                              int64_t timestamp_nanosec)
    {
        if (rgb_img.empty())
        {
            // return false when img is empty
            return std::async([]()
                              { return false; });
        }

        // Reprocess
        Eigen::Matrix3f
            transform_matrix; // transform matrix from resized image to source image.
        cv::Mat resized_img = letterbox(this->option_, rgb_img, transform_matrix);

        // Start async detect
        return std::async(std::launch::async, &SeekDetector::processCallback, this,
                          resized_img, transform_matrix, timestamp_nanosec, rgb_img);
    }

    SeekDetectorOption &SeekDetector::getOption()
    {
        return this->option_;
    }

    // void SeekDetector::setNmsCallback(NmsCallbackType callback)
    // {
    //     if (option_.getNmsSwitch() == true)
    //     {
    //         infer_nms_callback_ = callback;
    //     }
    //     else
    //     {
    //         infer_nms_callback_ = nullptr;
    //         FYT_WARN("seek_detector", "NMS is not enabled in SeekDetectorOption.");
    //         throw std::runtime_error("NMS is not enabled in SeekDetectorOption.");
    //     }
    // }

    void SeekDetector::setCallback(CallbackType callback)
    {
        if(option_.getNmsSwitch() == false)
        {
            infer_callback_ = callback;
        }
        else
        {
            infer_callback_ = nullptr;
            FYT_WARN("seek_detector", "NMS is enabled in SeekDetectorOption.");
            throw std::runtime_error("NMS is enabled in SeekDetectorOption.");
        }
    }

    bool SeekDetector::processCallback(const cv::Mat resized_img,
                                       Eigen::Matrix3f transform_matrix,
                                       int64_t timestamp_nanosec,
                                       const cv::Mat &src_img)
    {
        // BGR->RGB, u8(0-255)->f32(0.0-1.0), HWC->NCHW
        // note: TUP's model no need to normalize

        cv::Mat blob = cv::dnn::blobFromImage(
            resized_img, 1. / 255., cv::Size(option_.getInputWidth(), option_.getInputHeight()), cv::Scalar(0, 0, 0), true);

        // Feed blob into input
        auto input_port = compiled_model_->input();
        ov::Tensor input_tensor(
            input_port.get_element_type(),
            ov::Shape(std::vector<size_t>{1, 3, (size_t)option_.getInputWidth(), (size_t)option_.getInputHeight()}), blob.ptr(0));

        // Start inference
        // Lock because of the thread race condition within the openvino library
        mtx_.lock();
        auto infer_request = compiled_model_->create_infer_request();
        infer_request.set_input_tensor(input_tensor);
        infer_request.infer();
        mtx_.unlock();

        auto output = infer_request.get_output_tensor();

        // Process output data
        auto output_shape = output.get_shape();
        // 4725 x 15 Matrix
        cv::Mat output_buffer(output_shape[1], output_shape[2], CV_32F,
                              output.data());

        // Parsed variables
        std::vector<SeekObject> objs_tmp, objs_result;
        std::vector<cv::Rect> rects;
        std::vector<float> scores;
        std::vector<int> indices;
        SeekObject obj_tmp;

        // Parse YOLO output
        generateUniqueProposal(this->option_, obj_tmp, output_buffer, transform_matrix, 0);

        if (this->infer_callback_ && (this->option_.getNmsSwitch() == false))
        {
            this->infer_callback_(obj_tmp, timestamp_nanosec, src_img);
            return true;
        }

        return false;
    }

} // namespace fyt::seek
