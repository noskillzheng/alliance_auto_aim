/**
 * @file include/utils/visualization.hpp
 * @brief Utility helper for Visualization.
 */

#pragma once

#include "interfaces/armor_in_camera.hpp"
#include "interfaces/armor_in_gimbal_control.hpp"
#include "interfaces/armor_in_image.hpp"
#include <opencv2/core/mat.hpp>
namespace world_exe::util::visualization {
/**
 * @brief 在图像上绘制识别出的装甲板。
 */
void draw_armor_in_image(const interfaces::IArmorInImage& image, cv::Mat& InputOutputArray);

/**
 * @brief 将三维装甲板投影到图像平面并绘制。
 */
void draw_armor_in_camera(const interfaces::IArmorInCamera& camera_armor,
    const cv::Mat& intrinsic_parameters, const cv::Mat& distortion_parameters,
    const std::vector<cv::Point3d>& points_in_armor_spacing, cv::Mat& in_out_mat);
/**
 * @brief 在云台坐标系下进行投影可视化。
 *
 * @param camera_armor 云台坐标系下装甲板
 * @param gimal_to_camera 云台到相机的外参
 */
void draw_armor_in_gimbal(
    const world_exe::interfaces::IArmorInGimbalControl& camera_armor, const cv::Mat& intrinsic_parameters,
    const cv::Mat& distortion_parameters, const std::vector<cv::Point3d>& points_in_armor_spacing,
    const Eigen::Affine3d gimal_to_camera, cv::Mat& in_out_mat);
}
