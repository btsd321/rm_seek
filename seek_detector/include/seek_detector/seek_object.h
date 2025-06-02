#ifndef SEEK_OBJECT_H
#define SEEK_OBJECT_H

// 3rd party
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
// project
#include "rm_utils/common.hpp"

namespace fyt::seek
{

    class BoxRect : public cv::Rect
    {
    public:
        BoxRect() = default;
        BoxRect(cv::Point2f center, float width, float height)
        {
            this->x = center.x - width / 2;
            this->y = center.y - height / 2;
            this->f32_width_ = width;
            this->f32_height_ = height;
            this->width = static_cast<int>(this->f32_width_);
            this->height = static_cast<int>(this->f32_height_);
            this->center_ = center;
        }

        BoxRect(float center_x, float center_y, float width, float height)
        {
            this->x = center_x - width / 2;
            this->y = center_y - height / 2;
            this->f32_width_ = width;
            this->f32_height_ = height;
            this->width = static_cast<int>(this->f32_width_);
            this->height = static_cast<int>(this->f32_height_);
            this->center_.x = center_x;
            this->center_.y = center_y;
        }

        cv::Point2f get_center() const
        {
            return center_;
        }

        float get_f32_width() const
        {
            return f32_width_;
        }

        float get_f32_height() const
        {
            return f32_height_;
        }

        void set_f32_width(float f32_width)
        {
            this->f32_width_ = f32_width;
        }

        void set_f32_height(float f32_height)
        {
            this->f32_height_ = f32_height;
        }

        // 将BoxRect中的点进行线性变换
        BoxRect& transform(Eigen::Matrix3f transform_matrix)
        {
            // 有3个点需要转换分别是((x,y)、(x+width,y+height)、(center_x,center_y))

            // Transform keypoints to raw image
            Eigen::Matrix<float, 3, 3> apex_norm;
            Eigen::Matrix<float, 3, 3> apex_dst;

            apex_norm << this->x, this->x + this->f32_width_, this->center_.x, 
                        this->y, this->y + this->f32_height_, this->center_.y, 
                        1, 1, 1;
            apex_dst = transform_matrix * apex_norm;

            this->x = apex_dst(0, 0);
            this->y = apex_dst(1, 0);
            this->f32_width_ = apex_dst(0, 1) - apex_dst(0, 0);
            this->f32_width_ = apex_dst(1, 1) - apex_dst(1, 0);
            this->width = static_cast<int>(this->f32_width_);
            this->height = static_cast<int>(this->f32_height_);
            this->center_.x = apex_dst(0, 2);
            this->center_.y = apex_dst(1, 2);
            return *this;
        }

    private:
        cv::Point2f center_;
        float f32_width_ = 0;
        float f32_height_ = 0;
    };

    class SeekObject
    {
    public:
        SeekObject() = default;
        SeekObject(BoxRect& box, float score, cv::Point2f& p1, cv::Point2f& p2, cv::Point2f& p3, cv::Point2f& p4)
            : poses_{p1, p2, p3, p4}, score_(score), box_(box)
        {
        }
        cv::Point2f get_point(int index) const
        {
            return poses_.at(index);
        }
        float get_score() const
        {
            return score_;
        }
        BoxRect get_box() const
        {
            return box_;
        }
        
        std::array<cv::Point2f, 4> get_points() const
        {
            return poses_;
        }

        // 将SeekObject中的点进行线性变换
        SeekObject& transform(Eigen::Matrix3f transform_matrix)
        {
            auto& p1 = poses_.at(0);
            auto& p2 = poses_.at(1);
            auto& p3 = poses_.at(2);
            auto& p4 = poses_.at(3);

            // 有4个点需要转换分别是(p1、p2、p3、p4)

            // Transform keypoints to raw image
            Eigen::Matrix<float, 3, 4> apex_norm;
            Eigen::Matrix<float, 3, 4> apex_dst;

            apex_norm << p1.x, p2.x, p3.x, p4.x,
                        p1.y, p2.y, p3.y, p4.y,
                        1, 1, 1, 1;
            
            apex_dst = transform_matrix * apex_norm;

            p1.x = apex_dst(0, 0);
            p2.x = apex_dst(0, 1);
            p3.x = apex_dst(0, 2);
            p4.x = apex_dst(0, 3);
            p1.y = apex_dst(1, 0);
            p2.y = apex_dst(1, 1);
            p3.y = apex_dst(1, 2);
            p4.y = apex_dst(1, 3);

            this->box_.transform(transform_matrix);
            return *this;
        }

    private:
        std::array<cv::Point2f, 4> poses_;
        float score_ = 0;
        BoxRect box_;

        friend class SeekObjectWithTimestamp;
    };

    class SeekObjectWithTimestamp : public SeekObject
    {
    public:
        SeekObjectWithTimestamp() = default;
        SeekObjectWithTimestamp(SeekObject &obj, int64_t time_stamp): time_stamp_(time_stamp)
        {
            this->poses_ = obj.poses_;
            this->score_ = obj.score_;
            this->box_ = obj.box_;
        }

        int64_t get_time_stamp() const
        {
            return time_stamp_;
        }
        void set_time_stamp(int64_t time_stamp)
        {
            this->time_stamp_ = time_stamp;
        }

    private:
        int64_t time_stamp_ = 0;
    };
}

#endif // SEEK_OBJECT_H