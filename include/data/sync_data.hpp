/**
 * @file include/data/sync_data.hpp
 * @brief Synchronization helper for Sync Data.
 */

#pragma once

#include "data/time_stamped.hpp"
#include <Eigen/Eigen>
#include <ctime>
namespace world_exe::data {
/**
 * @brief 相机-云台-枪口三者之间的同步数据。
 */
struct CameraGimbalMuzzleSyncData {
    /// 相机曝光（或采集）开始的时间戳。
    data::TimeStamp camera_capture_begin_time_stamp;
    /// 相机坐标系 -> 云台坐标系的仿射变换。
    Eigen::Affine3d camera_to_gimbal;
    /// 云台坐标系 -> 炮口/枪口坐标系的仿射变换。
    Eigen::Affine3d gimbal_to_muzzle;
};
}
