/**
 * @file include/core/event_bus.hpp
 * @brief Core infrastructure for Event Bus.
 */

#pragma once

#include <algorithm>
#include <atomic>
#include <execution>
#include <functional>
#include <string>
#include <unordered_map>

namespace world_exe::core {

/**
 * @brief 极简事件总线，按事件名分发数据，支持任意数据类型。
 *
 * 通过模板实例化实现不同 payload 的独立订阅表，默认在调用线程同步执行回调。
 */
class EventBus {

public:
    /**
     * @brief 发布状态
     *
     * - `OK`：事件正常投递
     * - `Block`：存在并发阻塞（当前实现未启用）
     * - `NoSubscriptor`：没有订阅者，事件被丢弃
     */
    enum class BusStatus : int { OK, Block = -1, NoSubscriptor = 1 };

    /**
     * @brief 发布事件
     *
     * @tparam TData 事件数据类型（按值传递）
     * @param event_name 事件名，用于匹配订阅者
     * @param data 承载的事件数据
     * @return BusStatus 描述是否成功触达订阅者
     */
    template <class TData>
    static world_exe::core::EventBus::BusStatus Publish(
        const std::string event_name, const TData& data) {
        return EventBusImpl<TData>::GetInstance().Publish(event_name, data);
    }

    /**
     * @brief 订阅事件
     *
     * @tparam TData 订阅的数据类型，需与发布侧一致
     * @param event_name 事件名
     * @param func 事件回调，收到数据时按注册顺序同步触发
     */
    template <typename TData>
    static void Subscript(const std::string& event_name, std::function<void(const TData&)>&& func) {
        return EventBusImpl<TData>::GetInstance().Subscript(event_name, std::move(func));
    }

private:
    template <typename TData> class EventBusImpl {
    private:
        EventBusImpl()                               = default;
        EventBusImpl(const EventBusImpl&)            = delete;
        EventBusImpl& operator=(const EventBusImpl&) = delete;

        std::unordered_map<std::string,
            std::tuple<std::atomic_bool, std::vector<std::function<void(const TData&)>>>>
            delegates = {};

    public:
        world_exe::core::EventBus::BusStatus Publish(
            const std::string& event_name, const TData& data) {
            if (!delegates.contains(event_name)) [[unlikely]] {
                return EventBus::BusStatus::NoSubscriptor;
            }
            auto& [flag, vec] = delegates[event_name];
            auto flag_val     = false;

            // if (!flag.compare_exchange_weak(flag_val, true))
            //     return EventBus::BusStatus::Block;
            // std::for_each(std::execution::par_unseq, vec.begin(), vec.end(),
            //     [&data](const auto& func) { func(data); });
            for (const auto& a : vec)
                a(data);
            // flag.store(false);
            return EventBus::BusStatus::OK;
        }
        void Subscript(const std::string event_name, std::function<void(const TData&)>&& func) {
            const auto& [iterator, create_falg] = delegates.try_emplace(event_name);
            auto& [flag, vec]                   = iterator->second;
            if (!create_falg) flag.store(false);
            vec.emplace_back(func);
        }
        static EventBusImpl<TData>& GetInstance() {
            static EventBusImpl<TData> instance_;
            return instance_;
        }
    };
};
} // namespace world_exe::core
