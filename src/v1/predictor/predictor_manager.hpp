/**
 * @file src/v1/predictor/predictor_manager.hpp
 * @brief Predictor component for Predictor Manager.
 */

#pragma once

#include "data/time_stamped.hpp"
#include "data/predictor_update_package.hpp"
#include "interfaces/target_predictor.hpp"

namespace world_exe::v1::predictor {
/**
 * @brief V1 版本的预测器管理器。
 */
class PredictorManager final : public world_exe::interfaces::ITargetPredictor {
public:
    PredictorManager();
    ~PredictorManager();

    /// 将新的观测包推送到内部各 predictor。
    void Update(std::shared_ptr<data::PredictorUpdatePackage> data);

    virtual std::shared_ptr<interfaces::IArmorInGimbalControl> Predict(
        const enumeration::ArmorIdFlag& id, const data::TimeStamp& time_stamp);

    std::shared_ptr<interfaces::IPredictor> GetPredictor(const enumeration::ArmorIdFlag& id) const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}
