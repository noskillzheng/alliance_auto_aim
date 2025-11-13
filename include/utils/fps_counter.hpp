/**
 * @file include/utils/fps_counter.hpp
 * @brief Utility helper for Fps Counter.
 */

#pragma once

#include <chrono>

namespace world_exe ::util {

/**
 * @brief 简单的帧率统计器，在给定窗口内累计次数。
 */
class FpsCounter {
public:
    /**
     * @brief 创建计数器
     * @param measurement_window 滑动窗口长度
     */
    explicit FpsCounter(
        std::chrono::steady_clock::duration measurement_window = std::chrono::seconds(1))
        : measurement_window_(measurement_window)
        , start_(std::chrono::steady_clock::now()) {}

    /**
     * @brief 记录一次事件，并在窗口结束时更新 FPS。
     * @return true 当窗口结束且 fps() 可用
     */
    bool count() {
        ++count_;

        auto now          = std::chrono::steady_clock::now();
        auto elapsed_time = now - start_;
        if (elapsed_time >= measurement_window_) {
            start_ = now;
            fps_   = double(count_) / std::chrono::duration<double>(elapsed_time).count();
            count_ = 0;
            return true;
        }

        return false;
    }

    /// 获取最近窗口统计到的 FPS。
    double fps() const { return fps_; }

private:
    std::chrono::steady_clock::duration measurement_window_;
    std::chrono::steady_clock::time_point start_;

    int64_t count_ = 0;  ///< 当前窗口内累计帧数
    double fps_    = 0;  ///< 最新一次计算得到的 FPS
};

} // namespace world_exe::util
