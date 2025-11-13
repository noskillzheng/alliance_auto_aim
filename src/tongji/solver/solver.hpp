/**
 * @file src/tongji/solver/solver.hpp
 * @brief Pose solving utilities for Solver.
 */

#pragma once

#include "interfaces/pnp_solver.hpp"

namespace world_exe::tongji::solver {

/**
 * @brief 使用多种手段求解装甲板 PnP，并转换到云台坐标系。
 */
class Solver final : public interfaces::IPnpSolver {
public:
    explicit Solver();
    ~Solver();

    /**
     * @brief 求解装甲板三维位置。
     */
    std::shared_ptr<world_exe::interfaces::IArmorInCamera> SolvePnp(
        std::shared_ptr<interfaces::IArmorInImage> armors) override;

    /**
     * @brief 设置相机到云台的外参。
     */
    void SetCamera2Gimbal(
        const Eigen::Matrix3d& R_camera2gimbal, const Eigen::Vector3d& t_camera2gimbal);

    /// 将相机坐标转换到云台系。
    auto Camera2Gimbal(const Eigen::Vector3d& xyz_in_camera) const -> const auto;
    /**
     * @brief 通过优化计算装甲板在云台坐标下的 yaw。
     */
    auto CalculateOptimizeYaw(const data::ArmorImageSpacing& armor_in_image,
        const Eigen::Vector3d& armor_xyz_in_gimbal, const double& gimbal_yaw,
        const double& initial_armor_yaw_in_gimbal) const -> const double;

    Solver(const Solver&)                = delete;
    Solver& operator=(const Solver&)     = delete;
    Solver(Solver&&) noexcept            = default;
    Solver& operator=(Solver&&) noexcept = default;

private:
    class Impl;

    std::unique_ptr<Impl> pimpl_;
};

}
