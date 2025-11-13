/**
 * @file src/tongji/identifier/identified_armor.hpp
 * @brief Target identifier module for Identified Armor.
 */

#pragma once

#include "enum/armor_id.hpp"
#include "interfaces/armor_in_image.hpp"
#include "data/time_stamped.hpp"
#include "util/index.hpp"

namespace world_exe::tongji::identifier {

/**
 * @brief `IArmorInImage` 的具体实现，按装甲 ID 聚合结果。
 */
class IdentifiedArmor final : public interfaces::IArmorInImage {
public:
    /**
     * @brief 使用识别结果构造，自动按 ID 分桶。
     */
    explicit IdentifiedArmor(const std::vector<data::ArmorImageSpacing>& armors) {
        for (const auto& armor : armors) {
            armors_[util::enumeration::GetIndex(armor.id)].emplace_back(armor);
        }
    }

    /// 返回识别结果的时间戳。
    const data::TimeStamp& GetTimeStamp() const override { return time_stamp_; }

    const std::vector<data::ArmorImageSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const override {
        return armors_[util::enumeration::GetIndex(armor_id)];
    }

    /**
     * @brief 将接口对象包装成具体类型（未实现）。
     */
    static IdentifiedArmor DecorateIArmorInImage(const interfaces::IArmorInImage& armor) {
        throw std::runtime_error("Not implemented");
    }

private:
    data::TimeStamp time_stamp_ { std::chrono::steady_clock::now().time_since_epoch() };
    std::array<std::vector<data::ArmorImageSpacing>, 8> armors_;
};
}
