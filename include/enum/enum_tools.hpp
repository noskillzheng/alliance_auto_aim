/**
 * @file enum_tools.hpp
 * @brief 枚举工具函数集
 * @details 提供枚举类型的常用操作函数,包括位标志判断和枚举名称获取
 * @author Alliance Algorithm Team
 * @date 2024
 */

#pragma once

#include <bit>
#include <cstdint>
#include <string_view>
#include <type_traits>

namespace world_exe::enumeration {

/**
 * @brief 判断枚举标志是否包含指定的位标志
 * @details
 * 使用位运算判断target枚举值是否包含compare指定的所有位标志。
 * 该函数是泛型函数,支持任何枚举类型,并根据枚举的底层类型大小
 * 自动选择对应的无符号整数类型进行位运算。
 *
 * 实现原理:
 * 1. 使用std::bit_cast将枚举值转换为对应的无符号整数类型
 * 2. 执行位与运算 (target & compare)
 * 3. 判断结果是否等于compare,相等则说明target包含compare的所有位
 *
 * 支持的枚举底层类型大小:
 * - 1字节 (uint8_t)
 * - 2字节 (uint16_t)
 * - 4字节 (uint32_t)
 * - 8字节 (uint64_t)
 *
 * @tparam FlagT 枚举类型,必须满足std::is_enum要求
 * @param target 目标枚举值,要检查的标志集合
 * @param compare 比较枚举值,要查找的标志位
 * @return true 如果target包含compare的所有位标志
 * @return false 如果target不包含compare的某些位标志
 *
 * @note 该函数使用constexpr,可在编译期求值
 * @note 使用std::bit_cast保证类型安全的位转换(C++20)
 * @warning compare应该是单个标志或标志组合,不要使用Unknow等特殊值
 *
 * 使用示例:
 * @code
 * ArmorIdFlag target = ArmorIdFlag::Hero;
 * ArmorIdFlag check = ArmorIdFlag::Hero;
 *
 * // 检查是否包含英雄标志
 * if (IsFlagContains(target, check)) {
 *     std::cout << "包含英雄标志" << std::endl;
 * }
 *
 * // 检查组合标志
 * uint32_t combined = static_cast<uint32_t>(ArmorIdFlag::Hero) |
 *                     static_cast<uint32_t>(ArmorIdFlag::Engineer);
 * ArmorIdFlag combinedFlag = static_cast<ArmorIdFlag>(combined);
 *
 * if (IsFlagContains(combinedFlag, ArmorIdFlag::Hero)) {
 *     std::cout << "组合标志包含英雄" << std::endl;
 * }
 * @endcode
 *
 * @see ArmorIdFlag 标志枚举定义
 */
template <typename FlagT>
    requires std::is_enum<FlagT>::value
constexpr const bool IsFlagContains(const FlagT& target, const FlagT& compare) {
    if constexpr (sizeof(FlagT) == sizeof(uint32_t))
        return (std::bit_cast<uint32_t>(target) & (uint32_t)compare) == (uint32_t)compare;
    else if (sizeof(FlagT) == sizeof(uint64_t))
        return (std::bit_cast<uint64_t>(target) & (uint64_t)compare) == (uint64_t)compare;
    else if (sizeof(FlagT) == sizeof(uint16_t))
        return (std::bit_cast<uint16_t>(target) & (uint16_t)compare) == (uint16_t)compare;
    else if (sizeof(FlagT) == sizeof(uint8_t))
        return (std::bit_cast<uint8_t>(target) & (uint8_t)compare) == (uint8_t)compare;
}

/**
 * @brief 获取枚举值的名称字符串
 * @details
 * 使用编译器内置宏在编译期提取枚举值的名称。该函数是零运行时开销的,
 * 所有字符串处理都在编译期完成。
 *
 * 实现原理:
 * 1. 利用编译器提供的函数签名宏(__PRETTY_FUNCTION__或__FUNCSIG__)
 * 2. 从函数签名中解析出枚举值的完整名称
 * 3. 提取最后的::之后的部分(去除命名空间前缀)
 * 4. 返回std::string_view(零拷贝,指向编译期字符串)
 *
 * 支持的编译器:
 * - GCC (使用__PRETTY_FUNCTION__)
 * - Clang (使用__PRETTY_FUNCTION__)
 * - MSVC (使用__FUNCSIG__)
 *
 * @tparam value 编译期常量枚举值
 * @return constexpr auto 返回std::string_view,指向枚举值的名称字符串
 *
 * @note 该函数完全在编译期执行,没有运行时开销
 * @note 返回的字符串视图指向静态存储区,生命周期是整个程序运行期
 * @warning 对于无效的枚举值,可能返回数字形式的字符串
 *
 * 使用示例:
 * @code
 * // 获取枚举名称
 * constexpr auto name1 = enum_name<ArmorIdFlag::Hero>();
 * std::cout << "枚举名称: " << name1 << std::endl;  // 输出: Hero
 *
 * constexpr auto name2 = enum_name<ArmorIdFlag::Engineer>();
 * std::cout << "枚举名称: " << name2 << std::endl;  // 输出: Engineer
 *
 * // 用于日志输出
 * #define LOG_ENUM(value) \
 *     std::cout << "枚举值: " << enum_name<value>() << std::endl
 *
 * LOG_ENUM(ArmorIdFlag::Sentry);  // 输出: Sentry
 *
 * // 编译期使用
 * static_assert(enum_name<ArmorIdFlag::Hero>() == "Hero");
 * @endcode
 *
 * @see ArmorIdFlag 标志枚举定义
 */
template <auto value> constexpr auto enum_name() {
    std::string_view name;
#if __GNUC__ || __clang__
    // GCC和Clang使用__PRETTY_FUNCTION__
    // 格式: constexpr auto world_exe::enumeration::enum_name() [with auto value = ...]
    name              = __PRETTY_FUNCTION__;
    std::size_t start = name.find('=') + 2;
    std::size_t end   = name.size() - 1;
    name              = std::string_view { name.data() + start, end - start };
    start             = name.rfind("::");
#elif _MSC_VER
    // MSVC使用__FUNCSIG__
    // 格式: auto __cdecl world_exe::enumeration::enum_name<...>(void)
    name              = __FUNCSIG__;
    std::size_t start = name.find('<') + 1;
    std::size_t end   = name.rfind(">(");
    name              = std::string_view { name.data() + start, end - start };
    start             = name.rfind("::");
#endif
    // 提取最后的::之后的部分(枚举值名称)
    return start == std::string_view::npos
        ? name
        : std::string_view { name.data() + start + 2, name.size() - start - 2 };
}

}  // namespace world_exe::enumeration
