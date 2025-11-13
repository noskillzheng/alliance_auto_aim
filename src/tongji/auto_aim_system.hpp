/**
 * @file src/tongji/auto_aim_system.hpp
 * @brief Auto-aim orchestration for Auto Aim System.
 */

#pragma once

#include <memory>
namespace world_exe::tongji {

/**
 * @brief 同济自瞄系统入口，聚合识别、追踪、火控等子模块。
 *
 * 通过隐藏实现类确保资源按顺序初始化，外部仅需调用 `build` 即可完成系统装配。
 */
class AutoAimSystem final {
public:
    /**
     * @brief 构建单例系统实例
     *
     * @param debug 是否启用调试模式（开启后会发布更多可视化/诊断事件）
     *
     * @note 若实例已存在则忽略后续调用。
     */
    static void build(bool debug);

    /**
     * @brief 显式构造系统，通常仅由 `build` 调用
     *
     * @param debug 是否输出调试事件
     */
    explicit AutoAimSystem(const bool& debug);
    ~AutoAimSystem();

    AutoAimSystem(const AutoAimSystem&)            = delete;
    AutoAimSystem& operator=(const AutoAimSystem&) = delete;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    static std::unique_ptr<AutoAimSystem> v2;
};
}
