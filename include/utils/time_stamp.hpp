/**
 * @file include/utils/time_stamp.hpp
 * @brief Utility helper for Time Stamp.
 */

#pragma once

#include "data/time_stamped.hpp"
#include <chrono>
namespace world_exe::util::time_stamp {
/**
 * @brief Functor，用于生成基于 steady_clock 的 TimeStamp。
 */
    struct SteadyClock{
        data::TimeStamp operator()() const{
            return data::TimeStamp{std::chrono::steady_clock::now().time_since_epoch()};
        }
    };
}
