/**
 * @file src/v1/pnpsolver/armor_pnp_solver.hpp
 * @brief Pose solving utilities for Armor Pnp Solver.
 */

#pragma once
#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>

#include "interfaces/armor_in_image.hpp"
#include "interfaces/pnp_solver.hpp"

namespace world_exe::v1::pnpsolver {

/**
 * @brief 基于 IPPE 的装甲板 PnP 求解器。
 */
class ArmorIPPEPnPSolver final : public interfaces::IPnpSolver {
public:
    ArmorIPPEPnPSolver(const std::vector<cv::Point3d>& LargeArmorObjectPointsOpencv,
        const std::vector<cv::Point3d>& NormalArmorObjectPointsOpencv);
    ~ArmorIPPEPnPSolver();
    void set_time_point(const std::chrono::nanoseconds& time_point);

    std::shared_ptr<world_exe::interfaces::IArmorInCamera> SolvePnp(
        std::shared_ptr<interfaces::IArmorInImage> armor) override;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    std::chrono::nanoseconds time_point_;
};
}
