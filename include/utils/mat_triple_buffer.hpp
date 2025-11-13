/**
 * @file include/utils/mat_triple_buffer.hpp
 * @brief Utility helper for Mat Triple Buffer.
 */

#pragma once

#include "data/mat_stamped.hpp"
#include "data/time_stamped.hpp"
#include "utils/time_stamp.hpp"

#include <atomic>
#include <concepts>
#include <opencv2/core/mat.hpp>
#include <optional>

namespace world_exe::util::memory {


/**
 * @brief 要求可调用对象返回 `data::TimeStamp`。
 */
template<typename F>
concept BinaryFunctor = requires(F f) {
    { f() } -> std::same_as<data::TimeStamp>;
};

/**
 * @brief 简单的三缓冲图像交换器（单生产者/单消费者）。
 *
 * 提供无锁交换，依靠原子指针循环使用三份 `MatStamped`。
 */
template<BinaryFunctor Func>
class MatTripleBuffer {
/// SPSC only
public:
    /**
     * @brief 创建缓冲区
     * @param timeFunc 时间戳生成器
     */
    explicit MatTripleBuffer(Func timeFunc) : func_(timeFunc) {}

    /**
     * @brief 创建使用 SteadyClock 的默认缓冲。
     */
    static MatTripleBuffer<time_stamp::SteadyClock> default_buffer(){
        return  MatTripleBuffer(time_stamp::SteadyClock{});
    }
    
    /**
     * @brief 写入一帧图像（覆盖最旧缓冲）。
     */
    void set(const cv::Mat& image)
    {
        buffer[ptr_set_].Load(image,func_());
        size_t old = ptr_get_.exchange(ptr_set_, std::memory_order_acq_rel);
        data_version.fetch_add(1, std::memory_order_release);
        ptr_set_ = old;
    }
    /**
     * @brief 获取最新帧，若无新数据则返回 nullopt。
     */
    std::optional<std::reference_wrapper<data::MatStamped>> get()
    {
        size_t current_version = data_version.load(std::memory_order_acquire);
        size_t expected = last_read_version.load(std::memory_order_acquire);

        if (current_version == expected) return std::nullopt; 

        size_t old = ptr_get_.exchange(ptr_occ_, std::memory_order_acq_rel);
        ptr_occ_ = old;
        return buffer[ptr_occ_];
    }

    const Func          func_;      ///< 时间戳生成器
    data::MatStamped    buffer[3];  ///< 三份图像缓冲

    std::atomic_uint8_t ptr_occ_ = 0; ///< 当前被占用的缓冲索引
    std::atomic_uint8_t ptr_get_ = 1; ///< 消费端可读取的索引
    std::atomic_uint8_t ptr_set_ = 2; ///< 生产端写入的索引
    
    std::atomic<size_t> data_version{0};     ///< 当前数据版本
    std::atomic<size_t> last_read_version{0};///< 消费端已读取的版本
};

}
