/**
 * @file src/v1/identifier/identifier.hpp
 * @brief Target identifier module for Identifier.
 */

#pragma once

#include "interfaces/identifier.hpp"
#include <memory>

namespace world_exe::v1::identifier {
/**
 * @brief V1 版本装甲板识别器，包装旧推理实现。
 */
class Identifier : public interfaces::IIdentifier {
public:
    Identifier(const std::string& model_path, const std::string& device,
        const int& image_width = 1440, const int& image_height = 1080);
    ~Identifier();

    // false 为蓝色 ， true 为红色
    void SetTargetColor(bool target_color);

    const std::tuple<const std::shared_ptr<interfaces::IArmorInImage>, enumeration::CarIDFlag>
    identify(const cv::Mat& input_image) override;
    void set_match_magnification_ratio(const double& ratio);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}
