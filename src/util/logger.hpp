/**
 * @file src/util/logger.hpp
 * @brief Utility helper for Logger.
 */

#pragma once

#include <spdlog/spdlog.h>

namespace world_exe::util::logger {
std::shared_ptr<spdlog::logger> logger();

} // namespace logger
