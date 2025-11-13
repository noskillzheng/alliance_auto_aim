/**
 * @file include/interfaces/target_predictor.hpp
 * @brief Predictor component for Target Predictor.
 */

#pragma once

#include "armor_in_gimbal_control.hpp"
#include "data/time_stamped.hpp"
#include "enum/armor_id.hpp"
#include "interfaces/predictor.hpp"
#include <ctime>
#include <memory>

namespace world_exe::interfaces {

/**
 * @brief 在线目标预测接口，通常封装多个 `IPredictor` 实例。
 *
 * 返回的结果通常是当前最可信的目标轨迹快照，而非静态记录。
 */
class ITargetPredictor {
public:
    /**
     * @brief 直接预测指定车辆在给定时间的装甲板集合。
     *
     * @param id 目标车辆 ID
     * @param time_stamp 目标时间戳
     */
    virtual std::shared_ptr<interfaces::IArmorInGimbalControl> Predict(
        const enumeration::ArmorIdFlag& id, const data::TimeStamp& time_stamp) = 0;

    /**
     * @brief 获取某车辆对应的持久化预测器对象。
     */
    virtual std::shared_ptr<IPredictor> GetPredictor(const enumeration::ArmorIdFlag& id) const = 0;

    virtual ~ITargetPredictor() = default;
};
}
