/**
 * @file include/interfaces/identifier.hpp
 * @brief Target identifier module for Identifier.
 */

#pragma once

#include "enum/car_id.hpp"
#include "interfaces/armor_in_image.hpp"
#include <memory>
#include <opencv2/core/mat.hpp>

namespace world_exe::interfaces {

/**
 * @brief 自瞄装甲板识别器接口。
 */
class IIdentifier {
public:
    /**
     * @brief 对输入图像进行识别，输出装甲板集合及主 ID。
     *
     * @param input_image 原始相机图像
     * @return 识别出的装甲板集合及其主要目标 ID
     */
    virtual const std::tuple<const std::shared_ptr<IArmorInImage>, enumeration::CarIDFlag> identify(
        const cv::Mat& input_image) = 0;

    virtual ~IIdentifier() = default;
};
}
