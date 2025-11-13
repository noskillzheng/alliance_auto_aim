/**
 * @file include/interfaces/sync_block.hpp
 * @brief Synchronization helper for Sync Block.
 */

#pragma once

#include "data/time_stamped.hpp"

namespace world_exe::interfaces {
/**
 * @brief 时间同步缓存接口，按时间戳存取传感器数据。
 *
 * @tparam T 承载的数据类型（例如传感器原始数据包）
 */
template <class T> class ISyncBlock {

public:
    /**
     * @brief 写入最新数据。
     *
     * @param camera_data 传感器或控制端采样所得数据
     */
    virtual void set_data(const T& camera_data) = 0;

    /**
     * @brief 按时间戳查询数据。
     *
     * @param timestamp 目标时间
     * @return (最接近的采样, 是否命中精准时间)
     */
    virtual std::tuple<T, bool> get_data(const data::TimeStamp& timestamp) = 0;

    virtual ~ISyncBlock() = default;
};
}
