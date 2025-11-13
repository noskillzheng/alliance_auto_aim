/**
 * @file src/tongji/identifier/armor_filter.hpp
 * @brief Target identifier module for Armor Filter.
 */

#pragma once

#include <ranges>
#include <unordered_set>
#include <utility>
#include <vector>

#include "data/armor_image_spaceing.hpp"
#include "enum/armor_id.hpp"
#include "enum/car_id.hpp"
#include "util/index.hpp"

namespace world_exe::tongji::identifier {

/**
 * @brief 装甲板过滤器：剔除指定 ID 或暂不可打的目标。
 */
class ArmorFilter {
public:
    explicit ArmorFilter()
        : invincible_armor_({ }) { }

    /**
     * @brief 根据黑名单过滤装甲板集合。
     */
    auto FilterArmor(std::vector<data::ArmorImageSpacing> const& armors) const {
        // 25赛季没有5号装甲板
        // 不打前哨站
        // 过滤掉刚复活无敌的装甲板
        auto filtered = armors | std::views::filter([&](const auto& armor) {
            return armor.id != enumeration::ArmorIdFlag::InfantryV
                && armor.id != enumeration::ArmorIdFlag::Outpost
                && !invincible_armor_.count(armor.id);
        });

        return std::vector<data::ArmorImageSpacing>(filtered.begin(), filtered.end());
    }

    /**
     * @brief 根据当前无敌车辆 ID 更新过滤列表。
     */
    void Update(enumeration::CarIDFlag ids) {
        invincible_armor_.clear();
        for (auto id : util::enumeration::ExpandArmorIdFlags(ids)) {
            invincible_armor_.insert(std::move(id));
        }
    }

private:
    std::unordered_set<enumeration::ArmorIdFlag> invincible_armor_; ///< 当前不可攻击的装甲板
};
}
