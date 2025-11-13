/**
 * @file include/data/mat_stamped.hpp
 * @brief Data definitions for Mat Stamped.
 */

#pragma once

#include "data/time_stamped.hpp"

#include <ctime>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>

namespace world_exe::data {
/**
 * @brief 带时间戳的图像帧封装。
 */
struct MatStamped {
    /// 帧对应的时间戳。
    data::TimeStamp stamp{};
    /// 图像数据（共享 OpenCV 管理的引用计数）。
    cv::Mat mat {};

    MatStamped()                                = default;
    MatStamped(const MatStamped&)               = delete;
    MatStamped& operator=(const MatStamped&)    = delete;

    /**
     * @brief 使用右值图像构造。
     *
     * @param image 待移动的图像
     * @param stamp 关联时间戳
     */
    MatStamped(cv::Mat&& image, data::TimeStamp stamp)
    {
        mat = image;
        stamp = stamp;
    }

    /**
     * @brief 复制载入一帧图像。
     *
     * @param image 输入图像
     * @param now 时间戳
     */
    void Load(const cv::Mat& image, data::TimeStamp now)
    {
        image.copyTo(mat);
        stamp = now;
    }
};
}
