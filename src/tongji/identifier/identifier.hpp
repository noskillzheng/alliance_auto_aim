/**
 * @file src/tongji/identifier/identifier.hpp
 * @brief Target identifier module for Identifier.
 */

#pragma once

#include "interfaces/identifier.hpp"

namespace world_exe::tongji::identifier {

/// 相机识别到的灯条颜色。
enum Color { red, blue, extinguish, purple };

/**
 * @brief 灯条几何特征，包含位置、角度与形状信息。
 */
struct Lightbar {
    std::size_t id;
    Color color;
    cv::Point2f center, top, bottom, top2bottom;
    std::vector<cv::Point2f> points;
    double angle, angle_error, length, width, ratio;
    cv::RotatedRect rotated_rect;

    Lightbar(const cv::RotatedRect& rotated_rect, std::size_t id)
        : id(id)
        , rotated_rect(rotated_rect) {
        std::vector<cv::Point2f> corners(4);
        rotated_rect.points(&corners[0]);
        std::sort(corners.begin(), corners.end(),
            [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });

        this->center     = rotated_rect.center;
        this->top        = (corners[0] + corners[1]) / 2.;
        this->bottom     = (corners[2] + corners[3]) / 2.;
        this->top2bottom = bottom - top;

        this->points.emplace_back(top);
        this->points.emplace_back(bottom);

        this->width       = cv::norm(corners[0] - corners[1]);
        this->angle       = std::atan2(top2bottom.y, top2bottom.x);
        this->angle_error = std::abs(angle - M_PI / 2);
        this->length      = cv::norm(top2bottom);
        this->ratio       = length / width;
    }
};

/**
 * @brief Tongji 版本的图像识别器入口。
 */
class Identifier final : public interfaces::IIdentifier {
public:
    explicit Identifier(const std::string& config_path, const std::string& save_path,
        const bool& debug = true, const bool& record = false);
    ~Identifier();

    /**
     * @brief 识别输入图像，返回装甲板集合与主 ID。
     */
    const std::tuple<const std::shared_ptr<interfaces::IArmorInImage>, enumeration::CarIDFlag>
    identify(const cv::Mat& input_image) override;

    /**
     * @brief 设定当前跟踪的目标颜色。
     */
    void SetTargetColor(Color target_color);

    Identifier(const Identifier&)                = delete;
    Identifier& operator=(const Identifier&)     = delete;
    Identifier(Identifier&&) noexcept            = default;
    Identifier& operator=(Identifier&&) noexcept = default;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

}
