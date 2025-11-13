/**
 * @file src/tongji/fire_controller/aim_solver.hpp
 * @brief Fire-control component for Aim Solver.
 */

#pragma once

#include <cmath>
#include <ctime>
#include <memory>
#include <optional>

#include <vector>
#include <yaml-cpp/yaml.h>

#include "../predictor/car_predictor/car_predictor.hpp"
#include "aim_point_chooser.hpp"
#include "tongji/predictor/kalman_filter/extended_kalman_filter.hpp"
#include "tongji/predictor/kalman_filter/predict_model.hpp"
#include "trajectory.hpp"

namespace world_exe::tongji::fire_control {

/**
 * @brief 火控求解结果。
 */
struct AimSolution {
    bool valid;                    ///< 是否成功求解
    double yaw;                    ///< 目标 yaw (rad)
    double pitch;                  ///< 目标 pitch (rad)
    Eigen::Vector3d aim_point;     ///< 世界坐标系下的瞄准点
    double horizon_distance = 0;   ///< 水平距离（无人机专用）
};

/**
 * @brief 根据预测器结果和弹道模型求解云台指令。
 */
class AimingSolver {
public:
    using PredictorModel = predictor::EKFModel<11, 4>;
    using EKF            = predictor::ExtendedKalmanFilter<PredictorModel>;

    /**
     * @brief 构造求解器并加载配置。
     *
     * @param config_path 包含 yaw/pitch offset、弹速等的配置
     * @param gravity 重力加速度
     */
    AimingSolver(const std::string& config_path, const double& gravity = 9.7833)
        : aim_point_chooser_(std::make_unique<AimPointChooser>(config_path))
        , g_(gravity) {

        auto yaml     = YAML::LoadFile(config_path);
        yaw_offset_   = yaml["yaw_offset"].as<double>() / 57.3;   // degree to rad
        pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3; // degree to rad
        bullet_speed_ = yaml["bullet_speed"].as<double>();
    }

    /**
     * @brief 结合预测器快照与控制延迟求解最终瞄准姿态。
     *
     * @param snapshot 目标预测器快照
     * @param time_stamp 当前时间戳
     * @param control_delay_s 控制链路延迟
     */
    AimSolution SolveAimSolution(std::shared_ptr<interfaces::IPredictor> snapshot,
        data::TimeStamp time_stamp, const double& control_delay_s) {

        /**
         * @brief 迭代求解弹道飞行时间
         * @note 最多迭代 10 次，收敛条件：|Δfly_time| < 0.001 秒 = 1ms
         * @note 迭代原因：目标位置依赖飞行时间，飞行时间又依赖目标位置，需要迭代收敛
         * @note 收敛阈值 1ms 的理由：
         *       - 弹速约 15-30 m/s，1ms 对应位移仅 1.5-3 cm，对击打精度影响可忽略
         *       - 迭代通常 2-3 次即可收敛，1ms 是足够宽松的阈值
         */
        static constexpr int MAX_ITERATIONS = 10;
        static constexpr double CONVERGENCE_THRESHOLD_S = 0.001;  // 收敛阈值（秒）

        double prev_fly_time_s;
        Eigen::Vector3d final_aim_point;
        TrajectoryResult final_trajectory;
        bool converged = false;
        // HACK:不同击打点影响飞行时间的迭代，需要根据整车的状态（转速和坐标）来选择击打点，不得已将指针转换为派生类
        auto snapshot_derived = std::dynamic_pointer_cast<predictor::CarPredictor>(snapshot);

        // 迭代求解：预测目标位置 → 计算弹道 → 更新飞行时间 → 重新预测
        for (int i = 0; i < MAX_ITERATIONS; ++i) {
            const auto& dt = control_delay_s + prev_fly_time_s;
            const auto& armors =
                snapshot->Predictor(time_stamp + data::TimeStamp::from_seconds(dt));

            const auto& aim_point = SelectPredictedAim(snapshot_derived->GetPredictedX(dt),
                armors->GetArmors(snapshot->GetId()), snapshot->GetId());
            if (!aim_point.has_value())
                return { false, std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN(), { },
                    0 }; // failed: no valid aim point
            const auto traj = SolveTrajectory(aim_point.value(), bullet_speed_);
            if (!traj.has_value())
                return { false, std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN(), { },
                    0 }; // failed: trajectory unsolvable

            // 检查收敛：相邻两次飞行时间差 < 1ms
            if (i > 0 && std::abs(traj->fly_time - prev_fly_time_s) < CONVERGENCE_THRESHOLD_S) {
                final_aim_point  = *aim_point;
                final_trajectory = *traj;
                converged        = true;
                break;
            }
            prev_fly_time_s = traj->fly_time;
        }
        if (!converged)
            return { false, std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(), { },
                0 }; // failed: trajectory did not converge

        const auto xyz     = final_aim_point;
        const double yaw   = std::atan2(xyz.y(), xyz.x()) + yaw_offset_;
        const double pitch = -(final_trajectory.pitch + pitch_offset_);
        return { true, yaw, pitch, final_aim_point };
    }

private:
    /**
     * @brief 选择最优装甲板并输出其中心点。
     */
    std::optional<Eigen::Vector3d> SelectPredictedAim(const EKF::XVec& ekf_x,
        const std::vector<data::ArmorGimbalControlSpacing>& armors, const CarIDFlag& id) const {

        const auto& [selectable, aim_point_in_gimbal] =
            aim_point_chooser_->ChooseAimArmor(ekf_x, armors, id);

        if (!selectable) return std::nullopt;
        return aim_point_in_gimbal.position;
    }

    /**
     * @brief 求解弹道轨迹及飞行时间。
     */
    std::optional<TrajectoryResult> SolveTrajectory(
        const Eigen::Vector3d& xyz, const double& bullet_speed) const {
        double d    = std::hypot(xyz.x(), xyz.y());
        auto result = TrajectorySolver::SolveTrajectory(bullet_speed, d, xyz.z(), g_);
        return result.solvable ? std::optional { result } : std::nullopt;
    }

    double yaw_offset_, pitch_offset_;
    double bullet_speed_;
    const double g_;

    std::unique_ptr<AimPointChooser> aim_point_chooser_;
};
}
