/**
 * @file src/tongji/fire_controller/fire_decision.hpp
 * @brief Fire-control component for Fire Decision.
 */

#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>

#include <yaml-cpp/yaml.h>

namespace world_exe::tongji::fire_control {
/**
 * @brief 云台指令（目标 yaw/pitch）。
 */
struct GimbalCommand {
    double yaw;
    double pitch;
};

/**
 * @brief 决定是否允许自动开火的判定器。
 */
class FireDecision {
public:
    /**
     * @brief 从配置加载容差和距离阈值。
     */
    explicit FireDecision(const std::string& config_path)
        : last_gimbal_command_({ std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN() }) {
        auto yaml         = YAML::LoadFile(config_path);
        auto_fire_        = yaml["auto_fire"].as<bool>();
        first_tolerance_  = yaml["first_tolerance"].as<double>() / 57.3;  // degree to rad
        second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3; // degree to rad
        judge_distance_   = yaml["judge_distance"].as<double>();
    }

    /**
     * @brief 判断当前帧是否允许开火。
     *
     * @param gimbal_yaw 当前云台 yaw
     * @param gimbal_command 目标指令
     * @param valid_target_pos 目标位置，用以区分远近
     */
    bool ShouldFire(const double& gimbal_yaw, GimbalCommand gimbal_command,
        const Eigen::Vector3d& valid_target_pos) {

        if (!auto_fire_) return false;
        const auto& tolerance = std::sqrt(valid_target_pos.x() * valid_target_pos.x()
                                    + valid_target_pos.y() * valid_target_pos.y())
                > judge_distance_
            ? second_tolerance_
            : first_tolerance_;

        if (std::abs(last_gimbal_command_.yaw - gimbal_yaw)
                < tolerance * 2 // 此时认为command突变不应该射击
            && std::abs(gimbal_yaw - last_gimbal_command_.yaw) < tolerance) {
            last_gimbal_command_ = gimbal_command;
            return true;
        }
        last_gimbal_command_ = gimbal_command;
        return false;
    }

private:
    bool auto_fire_;
    GimbalCommand last_gimbal_command_; ///< 最近一次下发的云台指令

    double first_tolerance_ { 5 };  // 近距离射击容差，degree
    double second_tolerance_ { 2 }; // 远距离射击容差，degree
    double judge_distance_ { 3 };   // 距离判断阈值
};

}
