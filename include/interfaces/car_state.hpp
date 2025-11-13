/**
 * @file include/interfaces/car_state.hpp
 * @brief Interface abstraction for Car State.
 */

#pragma once

#include "enum/car_id.hpp"
#include <opencv2/core/mat.hpp>

namespace world_exe::interfaces {

/**
 * @brief 负责确定自瞄系统中各车辆的可攻击状态。
 */
class ICarState {

public:
    /**
     * @brief 返回当前允许开火的车辆集合。
     */
    virtual const enumeration::CarIDFlag& GetAllowdToFires() const = 0;

    virtual ~ICarState() = default;
};
}
