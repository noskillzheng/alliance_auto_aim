/**
 * @file src/v1/sync/syncer.hpp
 * @brief Synchronization helper for Syncer.
 */

#pragma once

#include "data/sync_data.hpp"
#include "data/predictor_update_package.hpp"
#include <chrono>
#include <interfaces/sync_block.hpp>
#include <memory>
#include <opencv2/core/mat.hpp>

namespace world_exe::v1 {
/**
 * @brief V1 的传感器同步器，实现 `ISyncBlock`。
 */
class Syncer final : interfaces::ISyncBlock<data::CameraGimbalMuzzleSyncData> {
public:
    Syncer(std::chrono::seconds time_to_hold,long tolerable_ns = 4e6);
    ~Syncer();

    /// 存入新的同步数据。
    void set_data(const data::CameraGimbalMuzzleSyncData& camera_data);

    /// 取出与时间戳最匹配的数据。
    std::tuple<data::CameraGimbalMuzzleSyncData, bool> get_data(const data::TimeStamp& timestamp);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
    std::shared_ptr<data::PredictorUpdatePackage> last_;
};
}
