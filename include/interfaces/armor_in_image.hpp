/**
 * @file include/interfaces/armor_in_image.hpp
 * @brief Interface abstraction for Armor In Image.
 */

#pragma once

#include "data/armor_image_spaceing.hpp"
#include "enum/armor_id.hpp"
#include "data/time_stamped.hpp"
#include <opencv2/core/core.hpp>
#include <vector>

namespace world_exe::interfaces {

/**
 * @brief 某一时刻相机图像坐标系下的装甲板集合。
 *
 * @warning ArmorImageSpacing 虽为二维描述，但可视为以相机光轴为 X、光心为原点的
 * x-forward y-up 实体坐标，便于后续 PnP 计算。
 */
class IArmorInImage {

public:
    /// 获取时间戳，标志其内容装甲板的准确时间点
    virtual const data::TimeStamp& GetTimeStamp() const = 0;

    /**
     * @brief 获取某个车辆ID的装甲板集合
     *
     * @param armor_id 要查找的车辆ID
     * @return const std::vector<data::ArmorImageSpacing>&
     */
    virtual const std::vector<data::ArmorImageSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const = 0;

    virtual ~IArmorInImage() = default;
};
}
