/**
 * @file src/v1/fire_controller/fire_controller.hpp
 * @brief Fire-control component for Fire Controller.
 */

#pragma once

#include "data/fire_control.hpp"
#include "interfaces/fire_controller.hpp"
#include "interfaces/predictor.hpp"
#include <chrono>
#include <memory>
namespace world_exe::v1::fire_control {
/**
 * @brief V1 版本的火控实现，按预测器输出生成云台指令。
 */
class TracingFireControl final : public interfaces::IFireControl {
public:
    const data::FireControl CalculateTarget(const std::chrono::seconds& time_duration) const override;
    const enumeration::CarIDFlag GetAttackCarId() const override;
    void set_armor(const std::shared_ptr<interfaces::IArmorInGimbalControl>& armors);
    void SetPredictor(const std::shared_ptr<interfaces::IPredictor>& predictor);
    void SetTargetCarID(const enumeration::CarIDFlag& tracing_id);
    void SetTimeStamp(const time_t& time);
    TracingFireControl(
        double control_delay_in_second, double velocity_begin, double gravity = 9.81);

    ~TracingFireControl();

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}
