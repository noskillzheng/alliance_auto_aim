/**
 * @file car_id.hpp
 * @brief 车辆ID类型定义
 * @details 为车辆ID提供类型别名,与装甲板ID保持一致的接口
 * @author Alliance Algorithm Team
 * @date 2024
 */

#pragma once

#include "./armor_id.hpp"

namespace world_exe::enumeration {

/**
 * @brief 车辆ID标志类型别名
 * @details
 * CarIDFlag是ArmorIdFlag的类型别名,用于在车辆识别场景中使用。
 * 这种设计使得代码语义更清晰,同时保持与装甲板ID系统的兼容性。
 *
 * 在RoboMaster比赛中,车辆和装甲板使用相同的ID系统进行标识,
 * 因此可以共享同一套枚举定义。使用类型别名可以:
 * - 提高代码可读性(在车辆相关代码中使用CarIDFlag更符合语义)
 * - 保持类型安全(实际上是同一种类型)
 * - 简化维护(只需维护一套ID定义)
 *
 * @note 该类型别名完全等价于ArmorIdFlag,所有ArmorIdFlag的操作和工具函数都适用
 *
 * 使用示例:
 * @code
 * // 在装甲板识别代码中
 * ArmorIdFlag armor = ArmorIdFlag::Hero;
 *
 * // 在车辆跟踪代码中(语义更清晰)
 * CarIDFlag vehicle = CarIDFlag::Hero;
 *
 * // 两者可以互相赋值和比较
 * if (armor == vehicle) {
 *     // 匹配成功
 * }
 * @endcode
 *
 * @see ArmorIdFlag 查看完整的ID枚举定义
 * @see enum_tools.hpp 查看ID相关的工具函数
 */
typedef world_exe::enumeration::ArmorIdFlag CarIDFlag;

}  // namespace world_exe::enumeration
