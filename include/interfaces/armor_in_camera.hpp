/**
 * @file include/interfaces/armor_in_camera.hpp
 * @brief Interface abstraction for Armor In Camera.
 */

#pragma once

#include "data/armor_camera_spacing.hpp"
#include "data/time_stamped.hpp"
#include "enum/armor_id.hpp"
#include "data/time_stamped.hpp"
#include <opencv2/core/mat.hpp>


namespace world_exe::interfaces {

/**
 * @brief 表示某一确定时刻，相机坐标系下的装甲板集合。
 */
class IArmorInCamera {
public:
    /// 获取时间戳，标志装甲板数据对应的采样时刻。
    virtual const data::TimeStamp& GetTimeStamp() const = 0;

    /// 获取指定车辆 ID 的装甲板集合。
    virtual const std::vector<data::ArmorCameraSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const = 0;

    virtual ~IArmorInCamera() = default;
};
}
