/**
 * @file include/data/predictor_update_package.hpp
 * @brief Predictor component for Predictor Update Package.
 */

#pragma once

#include "data/sync_data.hpp"
#include "interfaces/armor_in_camera.hpp"

#include "data/time_stamped.hpp"
#include <memory>

namespace world_exe::data {

/**
 * @brief 追踪预测器的更新包，携带姿态解算结果及时间同步信息。
 */
struct PredictorUpdatePackage final {
public:
    PredictorUpdatePackage(const data::CameraGimbalMuzzleSyncData& data1,
        std::shared_ptr<world_exe::interfaces::IArmorInCamera> data2)
        : data1_(data1)
        , data2_(std::move(data2)) { }
    PredictorUpdatePackage()  = delete;
    ~PredictorUpdatePackage() = default;

    /**
     * @brief 传感器获取画面时的时间戳。
     */
    const data::TimeStamp& GetTimeStamp() const { return data1_.camera_capture_begin_time_stamp; };

    /**
     * @brief 求解好的三维装甲板集合。
     */
    std::shared_ptr<world_exe::interfaces::IArmorInCamera> GetArmors() const { return data2_; };

    /**
     * @brief 相机到世界/云台坐标系的仿射变换。
     */
    Eigen::Affine3d GetCameraToWorld() const { return data1_.camera_to_gimbal; };

private:
    const data::CameraGimbalMuzzleSyncData& data1_;
    const std::shared_ptr<world_exe::interfaces::IArmorInCamera> data2_;
};
}
