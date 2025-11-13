/**
 * @file include/core/system_factory.hpp
 * @brief Core infrastructure for System Factory.
 */

#pragma once

#include "enum/system_version.hpp"

namespace world_exe::core {

/**
 * @brief 根据系统版本装配不同的自瞄组件组合。
 */
class SystemFactory {
public:
    /**
     * @brief 构建指定版本的系统
     *
     * @param version 需要加载的系统版本号，决定参数、算法及管线差异
     */
    static void Build(const enumeration::SystemVersion& version);
};
}
