#ifndef SEEK_POSE_ESTIMATOR_H
#define SEEK_POSE_ESTIMATOR_H

#include <rm_utils/math/extended_kalman_filter.hpp>
#include <array>
#include <opencv2/core.hpp>

namespace fyt::seek
{
    enum class MotionModel
    {
        CONSTANT_VELOCITY = 0, // Constant velocity
        STATIC = 1             // No velocity and rotation velocity
    };

    // X_N: (x, y, vx, vy) state dimension, Z_N: (x, y)measurement dimension
    constexpr int X_N = 4, Z_N = 2;
    constexpr int TARGET_POINT_NUM = 4;

    struct Predict
    {
        explicit Predict(double dt, MotionModel model = MotionModel::CONSTANT_VELOCITY)
            : dt(dt), model(model) {}

        template <typename T>
        void operator()(const T state[X_N], T state_predicted[X_N])
        {
            if (model == MotionModel::CONSTANT_VELOCITY)
            {
                state_predicted[0] = state[0] + state[2] * dt; // x
                state_predicted[1] = state[1] + state[3] * dt; // y
                state_predicted[2] = state[2];                  // vx
                state_predicted[3] = state[3];                  // vy
            }
            else if(model == MotionModel::STATIC)
            {
                // 显式设置 ceres::Jet 的值部分和导数部分
                state_predicted[2].a = 0.;     // vx 的值
                state_predicted[2].v.setZero(); // vx 的导数

                state_predicted[3].a = 0.;     // vy 的值
                state_predicted[3].v.setZero(); // vy 的导数
            }
            else
            {
                throw std::invalid_argument("Invalid motion model");
            }
        }

        double dt;
        MotionModel model;
    };

    struct Measure
    {
        template <typename T>
        void operator()(const T state[X_N], T measurement[Z_N])
        {
            measurement[0] = state[0]; // x
            measurement[1] = state[1]; // y
        }
    };

    class SeekPoseEstimator
    {
    public:
        SeekPoseEstimator(Eigen::Matrix4d Q, Eigen::Matrix2d R, double dt);

        // 更新滤波器
        void update(const std::array<cv::Point2f, TARGET_POINT_NUM> &target_points);

        // 获取滤波后的点
        std::array<cv::Point2f, TARGET_POINT_NUM> getFilteredPoints() const;

        void setFirstState(const std::array<cv::Point2f, TARGET_POINT_NUM> &first_points);

    private:
        // 定义滤波器类型
        using SeekObjectEKF = ExtendedKalmanFilter<X_N, Z_N, Predict, Measure>;
        // 每个点的滤波器
        std::unique_ptr<std::vector<SeekObjectEKF>> pfilters_;
        std::array<cv::Point2f, TARGET_POINT_NUM> filtered_points_;

        // 可调的 Q 和 R 矩阵
        Eigen::Matrix4d process_noise_cov_;     // Q 矩阵
        Eigen::Matrix2d measurement_noise_cov_; // R 矩阵
    };
}

#endif // SEEK_POSE_ESTIMATOR_H