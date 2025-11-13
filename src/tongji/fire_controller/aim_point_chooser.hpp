/**
 * @file src/tongji/fire_controller/aim_point_chooser.hpp
 * @brief Fire-control component for Aim Point Chooser.
 */

#pragma once

#include "data/armor_gimbal_control_spacing.hpp"
#include "enum/car_id.hpp"
#include "util/math.hpp"

#include <yaml-cpp/yaml.h>

namespace world_exe::tongji::fire_control {

using CarIDFlag = enumeration::CarIDFlag;

/**
 * @brief 装甲板选择器，根据 EKF 状态和旋转角挑选最优打击点。
 */
class AimPointChooser {
public:
    /**
     * @brief 读取配置初始化进入/离开角阈值。
     * @param config_path YAML 配置文件
     */
    AimPointChooser(const std::string& config_path) {
        auto yaml = YAML::LoadFile(config_path);
        // 57.3 = 180/π，用于角度转弧度：rad = degree / 57.3
        static constexpr double DEG_TO_RAD = 1.0 / 57.3;
        comming_angle_ = yaml["comming_angle"].as<double>() * DEG_TO_RAD;
        leaving_angle_ = yaml["leaving_angle"].as<double>() * DEG_TO_RAD;
    }

    /**
     * @brief 根据预测状态和可见装甲板列表选择目标。
     *
     * @param ekf_x 预测状态向量
     * @param armors 当前帧可见装甲板
     * @param single_id 车辆 ID（包含前哨站特殊逻辑）
     * @return first: 是否命中，second: 选中的装甲板
     */
    std::pair<bool, data::ArmorGimbalControlSpacing> ChooseAimArmor(
        const Eigen::Vector<double, 11>& ekf_x,
        const std::vector<data::ArmorGimbalControlSpacing> armors, const CarIDFlag& single_id) {
        const auto armor_num = armors.size();
        int chosen_id        = -1;

        // 整车旋转中心的球坐标yaw
        const auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

        std::vector<std::tuple<int, double>> delta_angle_list;
        for (int i = 0; i < armor_num; i++) {
            auto delta_angle = util::math::clamp_pm_pi(
                util::math::get_yaw_from_quaternion(armors[i].orientation) - center_yaw);
            delta_angle_list.emplace_back(std::make_tuple(i, delta_angle));
        }
        std::sort(delta_angle_list.begin(), delta_angle_list.end(),
            [](const auto& a, const auto& b) { return std::get<1>(a) > std::get<1>(b); });

        // 角速度阈值常量定义
        static constexpr double ANGULAR_VELOCITY_THRESHOLD = 2.0;  // rad/s，小陀螺判定阈值
        static constexpr double NORMAL_SHOOT_RANGE_DEG     = 60.0; // 度，常规射击范围
        static constexpr double OUTPOST_COMING_ANGLE_DEG   = 70.0; // 度，前哨站进入角
        static constexpr double OUTPOST_LEAVING_ANGLE_DEG  = 30.0; // 度，前哨站离开角
        static constexpr double DEG_TO_RAD = 1.0 / 57.3;           // 角度转弧度系数

        // 判断是否为非小陀螺状态（角速度 ≤ 2 rad/s）
        if (std::abs(ekf_x[8]) <= ANGULAR_VELOCITY_THRESHOLD && single_id != CarIDFlag::Outpost) {
            /**
             * @brief 常规模式（非小陀螺）：选择射击范围内的装甲板
             * @note 射击范围：±60°，超出此范围弹道偏差过大，不予考虑
             * @note 锁定机制：若范围内有多个装甲板，锁定最靠近中心的一个，避免频繁切换
             */
            std::vector<int> id_list;
            for (const auto& [id, delta_angle] : delta_angle_list) {
                if (std::abs(delta_angle) > NORMAL_SHOOT_RANGE_DEG * DEG_TO_RAD) continue;
                id_list.emplace_back(id);
            }

            if (id_list.size() == 1) {
                chosen_id = id_list[0];
                lock_id_  = -1;
            } else if (id_list.size() > 1) {
                // 未处于锁定模式时，选择 delta_angle 绝对值较小的装甲板，进入锁定模式
                if (lock_id_ == -1) lock_id_ = id_list[0];
                chosen_id = lock_id_;
            } else {
                lock_id_  = -1;
                chosen_id = lock_id_;
            }
        } else {
            /**
             * @brief 小陀螺模式：选择"正在出现"的装甲板
             * @note 前哨站参数特殊处理：
             *       - coming_angle = 70°（更大范围，因为前哨站移动慢）
             *       - leaving_angle = 30°（保守离开角，避免追踪已旋转走的装甲板）
             * @note 小陀螺原理：
             *       - 一侧装甲板不断"出现"（coming），另一侧不断"离开"（leaving）
             *       - 优先打"出现"的装甲板，命中概率更高
             */
            double coming_angle =
                (single_id == CarIDFlag::Outpost) ? OUTPOST_COMING_ANGLE_DEG * DEG_TO_RAD : comming_angle_;
            double leaving_angle =
                (single_id == CarIDFlag::Outpost) ? OUTPOST_LEAVING_ANGLE_DEG * DEG_TO_RAD : leaving_angle_;

            // 在小陀螺时，一侧的装甲板不断出现，另一侧的装甲板不断消失，显然前者被打中的概率更高
            //
            for (const auto& [id, delta_angle] : delta_angle_list) {
                if (std::abs(delta_angle) > coming_angle) continue;
                if ((ekf_x[7] > 0 && delta_angle < leaving_angle)
                    || (ekf_x[7] < 0 && delta_angle > -leaving_angle)) {
                    chosen_id = id;
                    break;
                }
            }
        }

        if (chosen_id == -1) {
            return { false, armors[std::get<0>(delta_angle_list.front())] };
        }

        return {
            true,
            armors[chosen_id],
        };
    }

private:
    /**
     * @brief 小陀螺模式下的"进入角"阈值（弧度）
     * @note 默认值：60° ≈ 1.047 rad
     * @note 含义：装甲板旋转进入视野时，在此角度范围内可以射击
     * @note 可通过配置文件调整
     */
    double comming_angle_ = 60.0 / 57.3;

    /**
     * @brief 小陀螺模式下的"离开角"阈值（弧度）
     * @note 默认值：20° ≈ 0.349 rad
     * @note 含义：装甲板即将旋转离开视野时，小于此角度则不再射击
     * @note 目的：避免打已经旋转走的装甲板，浪费弹药
     * @note 可通过配置文件调整
     */
    double leaving_angle_ = 20.0 / 57.3;

    /**
     * @brief 锁定状态下的装甲板索引
     * @note -1 表示未锁定
     * @note 锁定机制：当多个装甲板在射击范围内时，锁定一个避免频繁切换
     */
    int lock_id_ = -1;
};

}
