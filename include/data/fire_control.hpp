/**
 * @file include/data/fire_control.hpp
 * @brief Data definitions for Fire Control.
 */

#pragma once

#include "data/time_stamped.hpp"

#include <Eigen/Eigen>
#include <ctime>
#include <opencv2/core/core.hpp>

namespace world_exe::data {
/**
 * @brief 火控指令，包含云台目标姿态与开火许可。
 */
struct FireControl {
    /// 指令生成的参考时间。
    data::TimeStamp time_stamp;
    /// 目标云台方向向量（yaw, pitch, roll），单位 rad。
    Eigen::Vector3d gimbal_dir;
    /// 是否允许当前帧触发发射机构。
    bool fire_allowance = false;
};
}
