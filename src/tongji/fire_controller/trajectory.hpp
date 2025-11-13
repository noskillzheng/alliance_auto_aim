/**
 * @file src/tongji/fire_controller/trajectory.hpp
 * @brief Fire-control component for Trajectory.
 */

#pragma once

#include <cmath>
namespace world_exe::tongji::fire_control {

/**
 * @brief 弹道求解结果。
 */
struct TrajectoryResult {
    bool solvable = true;
    double fly_time; ///< 飞行时间，秒
    double pitch;    ///< 抬头角，弧度
};

/**
 * @brief 忽略空气阻力的简易弹道求解器。
 */
struct TrajectorySolver {
    /**
     * @brief 求解飞行时间与仰角。
     *
     * @param v0 初速 (m/s)
     * @param d 水平距离
     * @param h 垂直高度
     * @param g 重力加速度
     */
    static auto SolveTrajectory(const double& v0, const double& d, const double& h, const double& g)
        -> TrajectoryResult const {
        auto a     = g * d * d / (2 * v0 * v0);
        auto b     = -d;
        auto c     = a + h;
        auto delta = b * b - 4 * a * c;

        if (delta < 0) {
            return { .solvable = false, .fly_time = 0, .pitch = 0 };
        }

        auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
        auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
        auto pitch_1     = std::atan(tan_pitch_1);
        auto pitch_2     = std::atan(tan_pitch_2);
        auto t_1         = d / (v0 * std::cos(pitch_1));
        auto t_2         = d / (v0 * std::cos(pitch_2));
        return {
            .solvable = true,
            .fly_time = (t_1 < t_2) ? t_1 : t_2,
            .pitch    = (t_1 < t_2) ? pitch_1 : pitch_2,
        };
    }
};

}
