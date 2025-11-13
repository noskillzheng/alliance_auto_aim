/**
 * @file include/utils/cast.hpp
 * @brief Utility helper for Cast.
 */

#pragma once

#include "data/armor_camera_spacing.hpp"
#include "data/armor_image_spaceing.hpp"
#include <opencv2/core/mat.hpp>
namespace world_exe::util::cast {
/**
 * @brief 将三维装甲板投影到图像平面。
 *
 * @param armor3d 相机坐标系下的装甲板
 * @param intrinsic_parameters 相机内参
 * @param distortion_parameters 畸变系数
 * @param points_in_armor_spacing 装甲板局部坐标系的角点
 * @param out_armor_2d 输出的二维装甲板
 */
void armor_3d_camera_to_armor_2d_image(const data::ArmorCameraSpacing& armor3d,
    const cv::Mat& intrinsic_parameters, const cv::Mat& distortion_parameters,
    const std::vector<cv::Point3d>& points_in_armor_spacing, data::ArmorImageSpacing& out_armor_2d);

}
