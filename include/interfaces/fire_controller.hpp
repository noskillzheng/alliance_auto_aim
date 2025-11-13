/**
 * @file include/interfaces/fire_controller.hpp
 * @brief Fire-control component for Fire Controller.
 */

#pragma once

#include "data/fire_control.hpp"
#include "enum/car_id.hpp"
#include <chrono>
#include <ctime>

namespace world_exe::interfaces {

/**
 * @brief 火控系统接口，负责给出云台控制量与开火许可。
 */
class IFireControl {
public:
    /**
     * @brief 计算云台与发射系统控制量。
     *
     * @param time_duration 控制时间提前量，通常为当前时刻与传感器帧时间差
     * @return data::FireControl 目标云台角度与开火指令
     */
    virtual const data::FireControl CalculateTarget(const std::chrono::seconds& time_duration) const = 0;

    /**
     * @brief 返回当前火控系统锁定的车辆 ID。
     */
    virtual const enumeration::CarIDFlag GetAttackCarId() const = 0;

    virtual ~IFireControl() = default;
};
}
