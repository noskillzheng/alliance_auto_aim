/**
 * @file include/interfaces/armor_in_gimbal_control.hpp
 * @brief Interface abstraction for Armor In Gimbal Control.
 */

#pragma once

#include "data/armor_gimbal_control_spacing.hpp"
#include "enum/armor_id.hpp"
#include "data/time_stamped.hpp"
#include <opencv2/core/mat.hpp>

namespace world_exe::interfaces {

/**
 * @brief 某一时刻在云台/世界坐标系下的装甲板集合。
 */
class IArmorInGimbalControl {

public:
    /// 返回集合对应的时间戳，用于控制延迟补偿。
    virtual const data::TimeStamp& GetTimeStamp() const = 0;

    /// 获取指定车辆 ID 的装甲板集合。
    virtual const std::vector<data::ArmorGimbalControlSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const = 0;

    virtual ~IArmorInGimbalControl() = default;
};
}
