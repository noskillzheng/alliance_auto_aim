/**
 * @file include/data/time_stamped.hpp
 * @brief Data definitions for Time Stamped.
 */

#pragma once

#include <chrono>
#include <ctime>
namespace world_exe::data {
/**
 * @brief 基于 `std::chrono::steady_clock` 的简单时间戳封装。
 */
struct TimeStamp {

public:
    /// 创建零时间戳。
    inline TimeStamp()
        : stamp_() { }
    /// 拷贝构造。
    inline TimeStamp(const TimeStamp& stamp)
        : stamp_(stamp.stamp_) { }
    /// 使用纳秒构造。
    inline TimeStamp(const std::chrono::nanoseconds& stamp)
        : stamp_(stamp) { }
    /// 使用秒构造（转换为纳秒）。
    inline TimeStamp(const std::chrono::seconds& stamp)
        : stamp_(std::chrono::duration_cast<std::chrono::nanoseconds>(stamp)) { }

    /// 转换为秒，返回 double。
    inline constexpr double to_seconds() const { return std::chrono::duration_cast<std::chrono::duration<double>>(stamp_).count(); }
    /// 转换为纳秒，返回 double。
    inline constexpr double to_nanosec() const { return stamp_.count(); }

    auto operator<=>(const TimeStamp& other) const noexcept = default;
    /// 计算时间差。
    inline TimeStamp operator-(const TimeStamp& other) const noexcept { return TimeStamp { stamp_ - other.stamp_ }; }
    /// 计算时间和。
    inline TimeStamp operator+(const TimeStamp& other) const noexcept { return TimeStamp { stamp_ + other.stamp_ }; }
    /// 按比例缩放。
    inline TimeStamp operator*(const double& ratio) const noexcept { return from_nanosec(stamp_.count() * ratio); }
    /// 反比例缩放。
    inline TimeStamp operator/(const double& ratio) const noexcept { return from_nanosec(stamp_.count() / ratio); }

    /// 由秒创建时间戳。
    template <typename T> static inline TimeStamp from_seconds(const T time) {
        return TimeStamp { std::chrono::seconds(static_cast<long int>(time)) };
    }
    /// 由纳秒创建时间戳。
    template <typename T> static inline TimeStamp from_nanosec(const T time) {
        return TimeStamp { std::chrono::nanoseconds(static_cast<long int>(time)) };
    }

private:
    std::chrono::nanoseconds stamp_;

};
}
