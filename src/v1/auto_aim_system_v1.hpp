/**
 * @file src/v1/auto_aim_system_v1.hpp
 * @brief Auto-aim orchestration for Auto Aim System V1.
 */

#pragma once

#include <memory>
namespace world_exe::v1 {
/**
 * @brief 旧版自瞄系统入口，负责组装 V1 管线。
 */
class SystemV1 final{
public:
    /// 构建单例实例。
    static void build(const bool& debug);

    SystemV1(const bool& debug);
    ~SystemV1();
private:
    class Impl;

    SystemV1()                = delete;
    SystemV1(const SystemV1&) = delete;
    std::unique_ptr<Impl> instance_;

    static std::unique_ptr<SystemV1> v1;
};
}
