/**
 * @file src/tongji/predictor/car_predictor/car_predictor_manager.hpp
 * @brief Predictor component for Car Predictor Manager.
 */

#pragma once

#include <memory>

#include "data/predictor_update_package.hpp"
#include "data/time_stamped.hpp"
#include "enum/armor_id.hpp"
#include "interfaces/target_predictor.hpp"

namespace world_exe::tongji::predictor {

/**
 * @brief 目标预测器管理器，维护每辆车的 `CarPredictor`。
 */
class CarPredictorManager final : public interfaces::ITargetPredictor {
public:
    CarPredictorManager(const std::string& config_path, const double& timeout_sec = 0.1);
    ~CarPredictorManager();

    /**
     * @brief 预测指定车辆在时间戳下的装甲板。
     */
    std ::shared_ptr<interfaces ::IArmorInGimbalControl> Predict(
        const enumeration ::ArmorIdFlag& id, const data::TimeStamp& time_stamp) override;
    /**
     * @brief 获取内部持有的预测器实例。
     */
    std ::shared_ptr<interfaces::IPredictor> GetPredictor(
        const enumeration ::ArmorIdFlag& id) const override;

    /**
     * @brief 将新的同步数据和装甲观测推送给所有预测器。
     */
    void Update(std::shared_ptr<data::PredictorUpdatePackage> data);

    CarPredictorManager(const CarPredictorManager&)                = delete;
    CarPredictorManager& operator=(const CarPredictorManager&)     = delete;
    CarPredictorManager(CarPredictorManager&&) noexcept            = default;
    CarPredictorManager& operator=(CarPredictorManager&&) noexcept = default;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

}
