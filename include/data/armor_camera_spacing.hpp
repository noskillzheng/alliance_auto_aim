/**
 * @file include/data/armor_camera_spacing.hpp
 * @brief Data definitions for Armor Camera Spacing.
 */

#pragma once

#include "enum/armor_id.hpp"

#include <eigen3/Eigen/Eigen>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace world_exe::data {
/**
 * @brief 相机坐标系下装甲板位姿描述。
 */
struct ArmorCameraSpacing {
    /// 目标装甲板 ID（若未知则为 Unknow）。
    enumeration::ArmorIdFlag id = enumeration::ArmorIdFlag::Unknow;
    /// 装甲板中心在相机坐标系下的位置（X: 光轴, Z: 向上），单位 m。
    Eigen::Vector3d position;
    /// 表示从相机基座到装甲板局部坐标系（数字向前 / 向上）的旋转。
    Eigen::Quaterniond orientation;
};
}
