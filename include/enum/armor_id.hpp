/**
 * @file armor_id.hpp
 * @brief 装甲板ID枚举定义
 * @details 定义了RoboMaster比赛中所有机器人类型的装甲板ID标识
 * @author Alliance Algorithm Team
 * @date 2024
 */

#pragma once

#include <bit>
#include <cstdint>

namespace world_exe::enumeration {

/**
 * @brief 装甲板ID标志枚举
 * @details 使用位标志(bit flags)表示不同类型的机器人装甲板,支持位运算组合
 *
 * 该枚举用于标识和过滤装甲板类型,采用位掩码设计,可以高效地进行:
 * - 单个类型判断
 * - 多类型组合
 * - 类型过滤
 *
 * @note 使用32位无符号整数作为底层类型,支持最多32种不同的机器人类型
 * @warning Unknow使用-1的位转换表示,在进行位运算时需特别注意
 *
 * 使用示例:
 * @code
 * // 判断是否为英雄机器人
 * ArmorIdFlag flag = ArmorIdFlag::Hero;
 *
 * // 组合多个类型(需要配合工具函数)
 * uint32_t combined = static_cast<uint32_t>(ArmorIdFlag::Hero) |
 *                     static_cast<uint32_t>(ArmorIdFlag::Engineer);
 * @endcode
 */
enum class ArmorIdFlag : uint32_t {
    /** @brief 未知类型,使用-1的位转换表示全1 */
    Unknow = std::bit_cast<uint32_t>(-1),

    /** @brief 无装甲板/空值 */
    None   = 0b00000000,

    /** @brief 英雄机器人 (ID: 1) */
    Hero        = 0b00000001,

    /** @brief 工程机器人 (ID: 2) */
    Engineer    = 0b00000010,

    /** @brief 三号步兵机器人 (ID: 3) */
    InfantryIII = 0b00000100,

    /** @brief 四号步兵机器人 (ID: 4) */
    InfantryIV  = 0b00001000,

    /** @brief 五号步兵机器人 (ID: 5) */
    InfantryV   = 0b00010000,

    /** @brief 哨兵机器人 (ID: 6) */
    Sentry      = 0b00100000,

    /** @brief 基地 (ID: 7) */
    Base        = 0b01000000,

    /** @brief 前哨站 (ID: 8) */
    Outpost     = 0b10000000,

    /** @brief 机器人类型总数 */
    Count = 8
};

}  // namespace world_exe::enumeration
