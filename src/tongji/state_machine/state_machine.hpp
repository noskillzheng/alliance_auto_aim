/**
 * @file src/tongji/state_machine/state_machine.hpp
 * @brief State machine logic for State Machine.
 */

#pragma once

#include <ctime>
#include <memory>

#include "enum/car_id.hpp"
#include "interfaces/armor_in_image.hpp"
#include "interfaces/car_state.hpp"

namespace world_exe::tongji::state_machine {
/**
 * @brief 负责装甲板追踪状态转换，输出允许攻击的车辆列表。
 */
class StateMachine final : public interfaces::ICarState {
public:
    StateMachine();
    ~StateMachine();

    const enumeration ::CarIDFlag& GetAllowdToFires() const override;

    /**
     * @brief 依据识别结果更新状态机。
     */
    void Update(std::shared_ptr<interfaces::IArmorInImage> armors_in_image,
        const enumeration::CarIDFlag& invincible_armors,
        const std::chrono::milliseconds& duration_from_last_update);

    /// 主动设置丢失状态。
    void SetLostState();

    StateMachine(const StateMachine&)                = delete;
    StateMachine& operator=(const StateMachine&)     = delete;
    StateMachine(StateMachine&&) noexcept            = default;
    StateMachine& operator=(StateMachine&&) noexcept = default;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}
