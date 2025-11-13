/**
 * @file include/interfaces/pnp_solver.hpp
 * @brief Pose solving utilities for Pnp Solver.
 */

#pragma once

#include "interfaces/armor_in_camera.hpp"
#include "interfaces/armor_in_image.hpp"
namespace world_exe::interfaces {

/**
 * @brief PnP 求解器接口，通过二维装甲板推断三维位姿。
 */
class IPnpSolver {
public:
    /**
     * @brief 求解装甲板的 PnP 问题。
     * @param armor_in_image 二维装甲板数据
     * @return 三维装甲板集合
     */
    virtual std::shared_ptr<world_exe::interfaces::IArmorInCamera> SolvePnp(
        std::shared_ptr<interfaces::IArmorInImage>) = 0;

    virtual ~IPnpSolver() = default;
};
}
