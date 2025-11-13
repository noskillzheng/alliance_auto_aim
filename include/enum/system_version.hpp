/**
 * @file system_version.hpp
 * @brief 系统版本枚举定义
 * @details 定义了自瞄系统的版本标识符,用于版本管理和兼容性控制
 * @author Alliance Algorithm Team
 * @date 2024
 */

#pragma once

#include <cstdint>

namespace world_exe::enumeration {

/**
 * @brief 系统版本枚举
 * @details
 * 定义了自动瞄准系统的版本号标识,用于:
 * - 系统版本识别
 * - 配置文件兼容性检查
 * - 调试版本和发布版本的区分
 * - 不同版本功能特性的切换
 *
 * 版本号编码规则(32位无符号整数):
 * - 高16位: 主版本号 (Major Version)
 * - 低16位: 次版本号 (Minor Version)
 * - 次版本号 0x0000: 发布版本 (Release)
 * - 次版本号 0x0001: 调试版本 (Debug)
 *
 * 版本号示例:
 * - 0x00010000 = V1发布版 (1.0)
 * - 0x00010001 = V1调试版 (1.0-debug)
 * - 0x00020000 = V2发布版 (2.0)
 * - 0x00020001 = V2调试版 (2.0-debug)
 *
 * @note 使用32位整数便于版本比较和网络传输
 * @note 调试版本通常包含额外的日志输出和诊断功能
 * @warning 不同版本之间的配置文件和数据格式可能不兼容
 *
 * 使用示例:
 * @code
 * // 获取当前系统版本
 * SystemVersion current = SystemVersion::Default;
 *
 * // 版本判断
 * if (current == SystemVersion::V2) {
 *     // 使用V2版本的功能
 * }
 *
 * // 版本比较
 * uint32_t v1 = static_cast<uint32_t>(SystemVersion::V1);
 * uint32_t v2 = static_cast<uint32_t>(SystemVersion::V2);
 * if (v2 > v1) {
 *     std::cout << "V2是更新的版本" << std::endl;
 * }
 *
 * // 判断是否为调试版本
 * auto isDebug = [](SystemVersion ver) {
 *     return (static_cast<uint32_t>(ver) & 0xFFFF) == 0x0001;
 * };
 *
 * if (isDebug(SystemVersion::V1Debug)) {
 *     // 启用调试功能
 *     enableDebugLogging();
 * }
 *
 * // 提取主版本号
 * auto getMajor = [](SystemVersion ver) {
 *     return static_cast<uint32_t>(ver) >> 16;
 * };
 *
 * std::cout << "主版本号: " << getMajor(SystemVersion::V2) << std::endl;  // 输出: 2
 * @endcode
 *
 * @see enum_tools.hpp 查看版本相关的工具函数
 */
enum class SystemVersion : uint32_t
{
    /** @brief V1发布版本 (1.0) */
    V1       = 0x00010000,

    /** @brief V1调试版本 (1.0-debug),包含额外的调试信息 */
    V1Debug  = 0x00010001,

    /** @brief V2发布版本 (2.0) */
    V2       = 0x00020000,

    /** @brief V2调试版本 (2.0-debug),包含额外的调试信息 */
    V2Debug  = 0x00020001,

    /** @brief 占位版本,用于测试和开发 */
    Balabala = 0x00030000,

    /** @brief 默认版本,指向V1发布版 */
    Default  = V1
};

}  // namespace world_exe::enumeration
