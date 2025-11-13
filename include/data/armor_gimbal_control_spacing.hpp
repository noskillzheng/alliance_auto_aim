/**
 * @file include/data/armor_gimbal_control_spacing.hpp
 * @brief Data definitions for Armor Gimbal Control Spacing.
 */

#pragma once

#include "enum/armor_id.hpp"

#include <eigen3/Eigen/Eigen>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace world_exe::data {
/**
 * @brief 云台控制坐标系下的装甲板记录。
 */
struct ArmorGimbalControlSpacing {
    /// 装甲板所属车辆 ID。
    enumeration::ArmorIdFlag id = enumeration::ArmorIdFlag::Unknow;
    /// 云台/世界坐标系下的位置（米），X 指向枪口。
    Eigen::Vector3d position;
    /// 表示从云台基座到装甲板局部系的旋转。
    Eigen::Quaterniond orientation;
};
}
