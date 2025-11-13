/**
 * @file include/data/armor_image_spaceing.hpp
 * @brief Data definitions for Armor Image Spaceing.
 */

#pragma once

#include "enum/armor_id.hpp"
#include <opencv2/core/core.hpp>

namespace world_exe::data {
/**
 * @brief 图像平面中的装甲板角点及属性。
 */
struct ArmorImageSpacing {
    /// 装甲板 ID。
    enumeration::ArmorIdFlag id = enumeration::ArmorIdFlag::Unknow;
    /// 四个角点在图像坐标系下的像素坐标（左上起顺时针）。
    std::vector<cv::Point2d> image_points = { {}, {}, {}, {} };
    /// 是否为大装甲板（例如工程/英雄尺寸）。
    bool isLargeArmor;
};
}
