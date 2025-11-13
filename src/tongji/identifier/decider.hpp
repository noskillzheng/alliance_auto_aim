/**
 * @file src/tongji/identifier/decider.hpp
 * @brief Target identifier module for Decider.
 */



#pragma once

#include <memory>
#include <vector>

#include "data/armor_image_spaceing.hpp"
#include "enum/armor_id.hpp"

namespace world_exe::tongji::identifier {

/// 优先级配置模式。
enum PriorityMode { MODE_ONE = 1, MODE_TWO };

/// 单个装甲板在优先级列表中的序位。
enum class ArmorPriority {
    First = 1, //
    Second,    //
    Third,     //
    Forth,     //
    Fifth      //
};

/**
 * @brief 根据策略为装甲板列表打分并挑选最优目标。
 */
class Decider {
public:
    Decider(PriorityMode mode = PriorityMode::MODE_ONE);
    ~Decider();

    /**
     * @brief 选择当前帧的最佳装甲板 ID。
     */
    enumeration::ArmorIdFlag GetBestArmor(std::vector<data::ArmorImageSpacing>& armors) const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

}
