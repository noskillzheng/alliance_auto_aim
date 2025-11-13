/**
 * @file include/concepts/identifier.hpp
 * @brief Target identifier module for Identifier.
 */

#pragma once

#include "interfaces/identifier.hpp"
namespace world_exe::concepts {
/**
 * @brief 约束派生于 `IIdentifier` 且可构造的识别器类型。
 */
template <class T>
concept concept_identifier = requires(T t) {
    std::is_base_of_v<world_exe::interfaces::IIdentifier, T>;
    !std::is_abstract_v<T>;
};
}
