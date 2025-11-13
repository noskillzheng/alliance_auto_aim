/**
 * @file include/interfaces/sync_load.hpp
 * @brief Synchronization helper for Sync Load.
 */

#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <optional>

namespace world_exe::interfaces {

/**
 * @brief 简单的同步读写接口，常用于跨线程传递最新数据。
 *
 * @tparam T 承载的数据类型
 */
template <class T> class ISyncLoad {

public:
    /**
     * @brief 写入（或覆盖）最新数据。
     */
    virtual void Store(const T& data) = 0;
    /**
     * @brief 尝试读取数据。
     *
     * @return std::nullopt 表示尚无可读数据
     */
    virtual std::optional<T> Load()   = 0;

    virtual ~ISyncLoad() = default;
};
}
