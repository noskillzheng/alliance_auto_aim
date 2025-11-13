/**
 * @file src/tongji/fire_controller/fire_controller.hpp
 * @brief Fire-control component for Fire Controller.
 */

#pragma once

#include <chrono>
#include <memory>

#include "interfaces/car_state.hpp"
#include "interfaces/fire_controller.hpp"
#include "interfaces/target_predictor.hpp"

namespace world_exe::tongji::fire_control {

/**
 * @brief 火控执行器实现，桥接状态机、目标预测器与瞄准/发射策略。
 *
 * 该类封装参数读取、瞄准解算以及开火判定逻辑，通过 `IFireControl` 接口向上层暴露统一
 * 的云台指令与开火许可。
 */
class FireController final : public interfaces::IFireControl {
public:
    /**
     * @brief 构造火控系统
     *
     * @param config_path 火控配置文件路径（YAML），包含控制延迟、阈值等参数
     * @param state_machine 自瞄状态机，实现 `ICarState` 接口，提供当前可攻击目标
     * @param live_target_manager 目标预测器管理器，提供预测状态以供瞄准解算
     */
    FireController(const std::string& config_path,
        std::shared_ptr<interfaces::ICarState> state_machine,
        std::shared_ptr<interfaces::ITargetPredictor> live_target_manager);
    ~FireController();

    /**
     * @brief 计算云台姿态与开火许可
     *
     * @param time_duration 从追踪数据时间戳到当前的延迟，用于补偿控制量
     * @return data::FireControl 包含目标姿态和是否允许开火
     */
    const data ::FireControl CalculateTarget(
        const std::chrono::seconds& time_duration) const override;

    /**
     * @brief 获取当前锁定的目标车辆 ID
     *
     * @return enumeration::CarIDFlag 若当前允许开火则为锁定目标，否则返回 None
     */
    const enumeration ::CarIDFlag GetAttackCarId() const override;

    /**
     * @brief 更新最近一次测得的云台 yaw，用于闭环判定
     *
     * @param gimbal_yaw 云台当前偏航角（弧度）
     */
    void UpdateGimbalPosition(const double& gimbal_yaw);

    FireController(const FireController&)                = delete;
    FireController& operator=(const FireController&)     = delete;
    FireController(FireController&&) noexcept            = default;
    FireController& operator=(FireController&&) noexcept = default;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

}
