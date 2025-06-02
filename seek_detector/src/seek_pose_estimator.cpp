#include "seek_detector/seek_pose_estimator.h"

namespace fyt::seek
{
    SeekPoseEstimator::SeekPoseEstimator(Eigen::Matrix4d Q, Eigen::Matrix2d R, double dt)
    {
        try
        {
            this->process_noise_cov_ = Q;
            this->measurement_noise_cov_ = R;

            // 初始化 pfilters_
            pfilters_ = std::make_unique<std::vector<SeekObjectEKF>>();

            // 初始化每个点的滤波器
            for (size_t i = 0; i < TARGET_POINT_NUM; ++i)
            {
                // 初始化预测函数
                Predict predict_func(dt, MotionModel::CONSTANT_VELOCITY);

                // 初始化观测函数
                Measure measure_func;

                // 创建并添加滤波器
                pfilters_->emplace_back(
                    predict_func,
                    measure_func,
                    [this]() { return process_noise_cov_; }, // 使用成员变量 Q
                    [this](const SeekObjectEKF::MatrixZ1 &) { return measurement_noise_cov_; }, // 使用成员变量 R
                    SeekObjectEKF::MatrixXX::Identity() // 初始协方差矩阵 P
                );
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error initializing SeekPoseEstimator: " << e.what() << '\n';
            throw; // 重新抛出异常
        }
    }

    void SeekPoseEstimator::update(const std::array<cv::Point2f, TARGET_POINT_NUM> &target_points)
    {
        for (size_t i = 0; i < TARGET_POINT_NUM; ++i)
        {
            Eigen::Vector4d ekf_prediction = pfilters_->at(i).predict();
            Eigen::Matrix<double, Z_N, 1> ekf_measurement = Eigen::Matrix<double, Z_N, 1>(target_points.at(i).x, target_points.at(i).y);
            pfilters_->at(i).update(ekf_measurement);
            filtered_points_.at(i).x = ekf_prediction[0];
            filtered_points_.at(i).y = ekf_prediction[1];
        }
    }

    void SeekPoseEstimator::setFirstState(const std::array<cv::Point2f, TARGET_POINT_NUM> &first_points) {
        for (size_t i = 0; i < TARGET_POINT_NUM; ++i) {
            Eigen::Matrix<double, X_N, 1> initial_state;
            initial_state << first_points[i].x, first_points[i].y, 0.0, 0.0; // 初始速度设为 0
            pfilters_->at(i).setState(initial_state);
        }
    }

    std::array<cv::Point2f, TARGET_POINT_NUM> SeekPoseEstimator::getFilteredPoints() const
    {
        return filtered_points_;
    }
}