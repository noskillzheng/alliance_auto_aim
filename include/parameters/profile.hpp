/**
 * @file include/parameters/profile.hpp
 * @brief Parameter manager for Profile.
 */

#pragma once

#include <memory>
#include <opencv2/core/mat.hpp>
namespace world_exe::parameters {
/**
 * @brief 工业相机（Hikrobot）标定参数缓存。
 */
class HikCameraProfile {
public:
    /**
     * @brief 设置内参与畸变系数。
     */
    static void set_intrinsic_matrix(const double& fx, const double& fy, const double& cx,
        const double& cy, const double& k1, const double& k2, const double& k3);
    /// 设置图像宽高（像素）。
    static void set_width_height(const int& width, const int& height);
    /// 返回相机内参矩阵。
    static const cv::Mat& get_intrinsic_parameters();
    /// 返回畸变系数。
    static const cv::Mat& get_distortion_parameters();
    /// 返回图像宽度。
    static const int& get_width();
    /// 返回图像高度。
    static const int& get_height();

private:
    struct Impl;

    static std::unique_ptr<Impl> impl_;
};
}
