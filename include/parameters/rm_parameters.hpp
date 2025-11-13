/**
 * @file include/parameters/rm_parameters.hpp
 * @brief Parameter manager for Rm Parameters.
 */

#pragma once

#include <opencv2/core/types.hpp>
#include <vector>
namespace world_exe::parameters {
/**
 * @brief RoboMaster 装甲板尺寸及 3D 模型参数。
 */
class Robomaster {
public:
    /// 标准/大装甲板宽高（米），用于构建物体点。
    inline constexpr static const double NormalArmorWidth = 0.134, NormalArmorHeight = 0.056,
                                         LargerArmorWidth = 0.230, LargerArmorHeight = 0.056;

    /// 大装甲板在 ROS 坐标系中的顶点坐标。
    inline static const std::vector<cv::Point3d> LargeArmorObjectPointsRos = {
        cv::Point3d(0, 0.5 * LargerArmorWidth, 0.5 * LargerArmorHeight),
        cv::Point3d(0, -0.5 * LargerArmorWidth, 0.5 * LargerArmorHeight),
        cv::Point3d(0, -0.5 * LargerArmorWidth, -0.5 * LargerArmorHeight),
        cv::Point3d(0, 0.5 * LargerArmorWidth, -0.5 * LargerArmorHeight)
    };

    /// 普通装甲板在 ROS 坐标系中的顶点坐标。
    inline static const std::vector<cv::Point3d> NormalArmorObjectPointsRos = {
        cv::Point3d(0, 0.5 * NormalArmorWidth, 0.5 * NormalArmorHeight),
        cv::Point3d(0, -0.5 * NormalArmorWidth, 0.5 * NormalArmorHeight),
        cv::Point3d(0, -0.5 * NormalArmorWidth, -0.5 * NormalArmorHeight),
        cv::Point3d(0, 0.5 * NormalArmorWidth, -0.5 * NormalArmorHeight)
    };
    /// 大装甲板在 OpenCV 坐标系中的顶点坐标（Z 指向前）。
    inline static const std::vector<cv::Point3d> LargeArmorObjectPointsOpencv = {
        cv::Point3d(-0.5 * LargerArmorWidth, -0.5 * LargerArmorHeight, 0),
        cv::Point3d(0.5 * LargerArmorWidth, -0.5 * LargerArmorHeight, 0),
        cv::Point3d(0.5 * LargerArmorWidth, 0.5 * LargerArmorHeight, 0),
        cv::Point3d(-0.5 * LargerArmorWidth, 0.5 * LargerArmorHeight, 0)
    };

    /// 普通装甲板在 OpenCV 坐标系中的顶点坐标。
    inline static const std::vector<cv::Point3d> NormalArmorObjectPointsOpencv = {
        cv::Point3d(-0.5 * NormalArmorWidth, -0.5 * NormalArmorHeight, 0),
        cv::Point3d(0.5 * NormalArmorWidth, -0.5 * NormalArmorHeight, 0),
        cv::Point3d(0.5 * NormalArmorWidth, 0.5 * NormalArmorHeight, 0),
        cv::Point3d(-0.5 * NormalArmorWidth, 0.5 * NormalArmorHeight, 0)
    };
};
}
