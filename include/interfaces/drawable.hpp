/**
 * @file include/interfaces/drawable.hpp
 * @brief Interface abstraction for Drawable.
 */

#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

namespace world_exe::interfaces {

/**
 * @brief 支持可视化调试的对象抽象，提供绘制入口。
 */
class IDrawable {
public:
    /// 使用输入输出分离接口进行绘制。
    virtual void Draw(cv::InputArray, cv::OutputArray) = 0;
    /// 在原图上就地绘制。
    virtual void Draw(cv::InputOutputArray)            = 0;

    virtual ~IDrawable() = default;
};
}
