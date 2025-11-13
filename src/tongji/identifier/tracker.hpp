/**
 * @file src/tongji/identifier/tracker.hpp
 * @brief Target identifier module for Tracker.
 */

#pragma once

#include <ctime>

#include <memory>
#include <opencv2/core/types.hpp>
#include <vector>

#include "armor_filter.hpp"
#include "decider.hpp"
#include "enum/armor_id.hpp"
#include "identified_armor.hpp"

namespace world_exe::tongji::identifier {

/// 目标跟踪状态机状态。
enum class TrackState {
    Lost,      //
    Detecting, //
    Tracking,  //
    TempLost,  //
    Switching  //
};

/**
 * @brief 管理装甲板识别结果并维护跟踪状态机。
 */
class Tracker final {
    using ArmorInImage = world_exe::tongji::identifier::IdentifiedArmor;

public:
    Tracker()
        : armor_filter_(std::make_unique<identifier::ArmorFilter>())
        , decider_(std::make_unique<Decider>()) { }

    ~Tracker() = default;

    /**
     * @brief 选择当前应锁定的车辆 ID。
     *
     * @param armors_in_image 识别器输出
     * @param invincible_armors 无敌装甲 ID
     * @param duration_from_last_update 距离上次更新的时间
     */
    auto SelectTrackingTargetID(const std::shared_ptr<interfaces::IArmorInImage>& armors_in_image,
        const enumeration::CarIDFlag& invincible_armors,
        const std::chrono::milliseconds& duration_from_last_update) noexcept
        -> enumeration::ArmorIdFlag const {

        CheckCameraOffline(duration_from_last_update);
        armor_filter_->Update(invincible_armors);

        auto filtered_ids = enumeration::ArmorIdFlag::None;

        std::vector<data::ArmorImageSpacing> filtered_armors { };
        for (uint32_t i = 0; i < static_cast<int>(enumeration::ArmorIdFlag::Count); ++i) {
            auto id = static_cast<enumeration::ArmorIdFlag>(
                static_cast<uint32_t>(enumeration::ArmorIdFlag::Hero) << i);

            if (auto armors = armors_in_image->GetArmors(id); !armors.empty()) {
                // 对从图像识别到的装甲板进行过滤
                filtered_armors = armor_filter_->FilterArmor(armors);

                if (filtered_armors.empty()) continue;

                filtered_ids =
                    static_cast<enumeration::ArmorIdFlag>(static_cast<uint32_t>(filtered_ids)
                        | static_cast<uint32_t>(filtered_armors[0].id));
            }
        }

        UpdateState(filtered_ids != enumeration::ArmorIdFlag::None);

        if (state_ == TrackState::Tracking)
            tracking_car_id_ = decider_->GetBestArmor(filtered_armors);

        return tracking_car_id_;
    }

    /// 外部提示彻底丢失目标。
    void SetLostState() { state_ = TrackState::Lost; }

private:
    /// 状态机转移。
    void UpdateState(bool found) {
        switch (state_) {
        case TrackState::Lost: {
            if (found) {
                SetState(TrackState::Detecting);
                detect_count_ = 1;
            }
            break;
        }

        case TrackState::Detecting: {
            if (found) {
                detect_count_++;
                if (detect_count_ >= min_detect_count_) SetState(TrackState::Tracking);
            } else {
                detect_count_ = 0;
                SetState((pre_state_ == TrackState::Switching) ? TrackState::Switching
                                                               : TrackState::Lost);
            }
            break;
        }

        case TrackState::Tracking: {
            if (!found) {
                temp_lost_count_ = 1;
                SetState(TrackState::TempLost);
            }
            break;
        }

        case TrackState::Switching: {
            if (found) {
                SetState(TrackState::Detecting);
            } else {
                temp_lost_count_++;
                if (temp_lost_count_ > max_switch_count_) {
                    SetState(TrackState::Lost);
                    ResetTracking();
                };
            }
            break;
        }

        case TrackState::TempLost: {
            if (found) {
                SetState(TrackState::Tracking);
            } else {
                temp_lost_count_++;
                max_temp_lost_count_ = (tracking_car_id_ == enumeration::ArmorIdFlag::Outpost)
                    ? outpost_max_temp_lost_count_
                    : normal_max_temp_lost_count_;

                if (temp_lost_count_ > max_temp_lost_count_) {
                    SetState(TrackState::Lost);
                    ResetTracking();
                };
            }
            break;
        }
        }
    }

    /// 根据长时间无数据判断离线。
    void CheckCameraOffline(const std::chrono::milliseconds duration_from_last_update) {
        // if (state_ != TrackState::Lost && (duration_from_last_update > timeout_sec_);
        if ((duration_from_last_update > timeout_)) {
            SetState(TrackState::Lost);
            // std::cout << "I am lost QAQ" << std::endl;
        }
    }

    void SetState(TrackState new_state) {
        pre_state_ = state_;
        state_     = new_state;
    }

    void ResetTracking() { tracking_car_id_ = enumeration::CarIDFlag::None; }

    world_exe::enumeration::CarIDFlag tracking_car_id_ { enumeration::CarIDFlag::None };
    TrackState state_     = TrackState::Lost;
    TrackState pre_state_ = TrackState::Lost;

    std::unique_ptr<identifier::ArmorFilter> armor_filter_;
    std::unique_ptr<Decider> decider_;

    int detect_count_     = 0;  ///< 连续检测次数
    int temp_lost_count_  = 0;  ///< 临时丢失计数
    int max_temp_lost_count_ = 15;  ///< 最大临时丢失帧数（动态调整）

    /**
     * @brief 最小连续检测次数阈值
     * @note 从 Detecting 状态转换到 Tracking 状态需要连续检测到目标至少 5 帧
     * @note 目的：避免误检导致频繁切换状态，提高跟踪稳定性
     */
    const int min_detect_count_ = 5;

    /**
     * @brief 前哨站最大临时丢失帧数
     * @note 前哨站移动缓慢或静止，允许更长的丢失时间（75 帧）
     * @note 假设 30fps，75帧 ≈ 2.5秒，前哨站在此期间位置变化不大
     */
    const int outpost_max_temp_lost_count_ = 75;

    /**
     * @brief 常规目标最大临时丢失帧数
     * @note 常规车辆移动较快，丢失时间不能太长（15 帧）
     * @note 假设 30fps，15帧 = 0.5秒，超过此时间认为目标已真正丢失
     */
    const int normal_max_temp_lost_count_ = max_temp_lost_count_;

    /**
     * @brief 最大目标切换计数
     * @note 在 Temp_Lost 状态下，如果尝试切换目标的次数超过 200 次，
     *       则认为当前跟踪失败，重置为 Lost 状态
     * @note 目的：防止在丢失目标后频繁切换导致系统不稳定
     */
    const int max_switch_count_ = 200;

    /**
     * @brief 跟踪超时时间
     * @note 如果超过 100ms 未收到新的观测数据，认为跟踪超时
     * @note 用于处理相机掉帧或数据延迟的情况
     */
    const std::chrono::milliseconds timeout_ = std::chrono::milliseconds(100);
};

}
