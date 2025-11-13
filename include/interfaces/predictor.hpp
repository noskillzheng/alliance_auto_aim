/**
 * @file include/interfaces/predictor.hpp
 * @brief Predictor component for Predictor.
 */

#pragma once

#include "armor_in_gimbal_control.hpp"
#include "data/time_stamped.hpp"
#include "enum/armor_id.hpp"
#include <ctime>

namespace world_exe::interfaces {
/**
 * @brief 装甲板轨迹预测器抽象。
 *
 * 典型实现会维护状态向量 \f$x\f$ 及状态转移 \f{x' = Ax + Bu}\f，
 * 从而外推未来某时刻的目标姿态。
 */
class IPredictor {
public:
    /**
     * @brief 返回当前预测器负责的车辆 ID ，可用于调度多预测器。
     */
    virtual const enumeration::ArmorIdFlag& GetId() const = 0;
    /**
     * @brief 预测未来某个时间点的装甲板位姿。
     *
     * @param time_stamp 目标时间戳（通常为追踪基准时间 + 控制延迟）
     * @return 装甲板在云台坐标系下的集合
     */
    virtual std::shared_ptr<IArmorInGimbalControl> Predictor(
        const data::TimeStamp& time_stamp) const = 0;

    virtual ~IPredictor() = default;
};
}
