/**
 * @file include/concepts/pnp_solver.hpp
 * @brief Pose solving utilities for Pnp Solver.
 */

#pragma once
#include "../interfaces/pnp_solver.hpp"
#include <concepts>
namespace world_exe::concepts {
/**
 * @brief 约束派生于 `IPnpSolver` 且实现 `SolvePnp` 的类型。
 */
template <class T>
concept concept_pnp_solver = requires(T t, const world_exe::interfaces::IArmorInImage& data) {
    { t.SolvePnp(data) } -> std::same_as<const interfaces::IArmorInCamera&>;
    std::is_base_of_v<world_exe::interfaces::IPnpSolver, T>;
    !std::is_abstract_v<T>;
};
}
