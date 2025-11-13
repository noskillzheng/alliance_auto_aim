/**
 * @file src/v1/predictor/car/car_predictor.hpp
 * @brief Predictor component for Car Predictor.
 */

#pragma once

#include "car_predictor_ekf.hpp"
#include "data/time_stamped.hpp"
#include "enum/car_id.hpp"
#include "interfaces/predictor.hpp"

namespace world_exe::v1::predictor {
/**
 * @brief V1 版本的整车预测器，包装旧 EKF 实现。
 */
class CarPredictor final: public interfaces::IPredictor {
public:
    CarPredictor();
    ~CarPredictor();
    CarPredictor(const enumeration::CarIDFlag& id, const CarPredictEkf& ekf,
        const data::TimeStamp& create_time_stamp);

    /// 设置目标车辆 ID。
    void SetId(const enumeration::CarIDFlag& id);
    void SetEkf(const CarPredictEkf& ekf);
    void SetTimeStamp(const data::TimeStamp& time_stamp);

    const enumeration::ArmorIdFlag& GetId() const override;
    std::shared_ptr<interfaces::IArmorInGimbalControl> Predictor(
        const data::TimeStamp& time_stamp) const override;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}
