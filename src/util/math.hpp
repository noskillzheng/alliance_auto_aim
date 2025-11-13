/**
 * @file src/util/math.hpp
 * @brief Utility helper for Math.
 */

#pragma once

#include <Eigen/Eigen>

namespace world_exe::util::math {
/**
 * @brief 计算二维点的极角（方位角）
 * @param point 包含 x, y 成员的二维点
 * @return 极角（弧度），范围 [-π, π]
 * @note 使用 atan2 保证正确处理所有象限
 */
static constexpr double ratio(const auto& point) { return atan2(point.y, point.x); }

/**
 * @brief 将角度规范化到 (-2π, 2π) 区间
 * @param angle 输入角度（弧度）
 * @return 规范化后的角度（弧度），范围 (-2π, 2π)
 * @note tau (τ) = 2π，表示一个完整圆周
 */
static constexpr double clamp_pm_tau(auto&& angle) {
    while (angle >= 2 * std::numbers::pi)
        angle -= 2 * std::numbers::pi;
    while (angle <= -2 * std::numbers::pi)
        angle += 2 * std::numbers::pi;

    return angle;
}

/**
 * @brief 将角度规范化到 (-π, π] 区间
 * @param angle 输入角度（弧度）
 * @return 规范化后的角度（弧度），范围 (-π, π]
 * @note 用于将任意角度转换为等价的主值角（principal value）
 */
static constexpr double clamp_pm_pi(auto&& angle) {
    while (angle > std::numbers::pi)
        angle -= 2 * std::numbers::pi;
    while (angle <= -std::numbers::pi)
        angle += 2 * std::numbers::pi;

    return angle;
}

/**
 * @brief 从四元数提取偏航角（Yaw）
 * @param quaternion 输入四元数 (w, x, y, z)
 * @return 偏航角（弧度），绕 Z 轴旋转角度，范围 [-π, π]
 * @note 使用 ZYX 欧拉角顺序（先 Yaw，再 Pitch，最后 Roll）
 */
static inline double get_yaw_from_quaternion(const Eigen::Quaterniond& quaternion) {

    const double yaw =
        atan2(2.0 * (quaternion.w() * quaternion.z() + quaternion.x() * quaternion.y()),
            1.0 - 2.0 * (quaternion.y() * quaternion.y() + quaternion.z() * quaternion.z()));

    return yaw;
}

/**
 * @brief 从四元数提取俯仰角（Pitch）
 * @param quaternion 输入四元数 (w, x, y, z)
 * @return 俯仰角（弧度），绕 Y 轴旋转角度，范围 [-π/2, π/2]
 * @note 使用 ZYX 欧拉角顺序
 */
static inline double get_pitch_from_quaternion(const Eigen::Quaterniond& quaternion) {
    const double pitch =
        std::asin(2.0 * (quaternion.w() * quaternion.y() - quaternion.x() * quaternion.z()));

    return pitch;
}

/**
 * @brief 从四元数提取翻滚角（Roll）
 * @param quaternion 输入四元数 (w, x, y, z)
 * @return 翻滚角（弧度），绕 X 轴旋转角度，范围 [-π, π]
 * @note 使用 ZYX 欧拉角顺序
 */
static inline double get_roll_from_quaternion(const Eigen::Quaterniond& quaternion) {
    const double roll =
        std::atan2(2.0 * (quaternion.w() * quaternion.x() + quaternion.y() * quaternion.z()),
            1.0 - 2.0 * (quaternion.x() * quaternion.x() + quaternion.y() * quaternion.y()));

    return roll;
}

/**
 * @brief 将二维点坐标按指定角度旋转（坐标系重映射）
 * @param x 原 X 坐标
 * @param y 原 Y 坐标
 * @param delta_angle 旋转角度（弧度）
 * @return {新 X 坐标, 新 Y 坐标}
 * @note 等价于将坐标系逆时针旋转 delta_angle 角度
 */
static inline std::tuple<double, double> remap(
    const double& x, const double& y, const double& delta_angle) {
    if (x == 0 && y == 0) return { 0., 0. };
    const double distance = std::sqrt(x * x + y * y);
    double distance_angle { 0. };

    if (x == 0) {
        distance_angle = y > 0 ? std::numbers::pi / 2. : -std::numbers::pi / 2.;
    } else if (x > 0) {
        distance_angle = std::atan(y / x);
    } else if (x < 0) {
        distance_angle = std::atan(y / x) + std::numbers::pi;
    }
    distance_angle -= delta_angle;

    return { distance * std::cos(distance_angle), distance * std::sin(distance_angle) };
}

/**
 * @brief 将欧拉角转换为旋转矩阵
 * @param ypr 欧拉角向量 [yaw, pitch, roll]（弧度）
 * @return 3×3 旋转矩阵
 * @note 使用 ZYX 旋转顺序（内旋顺序：先绕 Z 轴 yaw，再绕 Y 轴 pitch，最后绕 X 轴 roll）
 * @note 外旋等价顺序：R = R_z(yaw) * R_y(pitch) * R_x(roll)
 */
static Eigen::Matrix3d euler_to_matrix(const Eigen::Vector3d& ypr) {
    double roll      = ypr[2];
    double pitch     = ypr[1];
    double yaw       = ypr[0];
    double cos_yaw   = cos(yaw);
    double sin_yaw   = sin(yaw);
    double cos_pitch = cos(pitch);
    double sin_pitch = sin(pitch);
    double cos_roll  = cos(roll);
    double sin_roll  = sin(roll);
    // clang-format off
    Eigen::Matrix3d R{
      {cos_yaw * cos_pitch, cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll, cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll},
      {sin_yaw * cos_pitch, sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll, sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll},
      {         -sin_pitch,                                cos_pitch * sin_roll,                                cos_pitch * cos_roll}
    };
    // clang-format on
    return R;
}

/**
 * @brief 将欧拉角转换为四元数
 * @param yaw_rad 偏航角（弧度）
 * @param pitch_rad 俯仰角（弧度）
 * @param roll_rad 翻滚角（弧度）
 * @return 单位四元数
 * @note 使用 ZYX 旋转顺序：q = q_yaw * q_pitch * q_roll
 */
static inline Eigen::Quaterniond euler_to_quaternion(
    const double& yaw_rad, const double& pitch_rad, const double& roll_rad) {
    Eigen::AngleAxisd rollAngle(roll_rad, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch_rad, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw_rad, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

/**
 * @brief 将四元数转换为欧拉角（通用版本，支持任意旋转顺序）
 * @param q 输入四元数
 * @param axis0 第一个旋转轴（0=X, 1=Y, 2=Z）
 * @param axis1 第二个旋转轴
 * @param axis2 第三个旋转轴
 * @param extrinsic 是否使用外旋（false=内旋）
 * @return 欧拉角向量 [angle0, angle1, angle2]（弧度），范围 (-π, π]
 * @note 支持所有12种欧拉角顺序（如 XYZ, ZYX, ZXZ 等）
 * @note 自动处理万向锁（Gimbal Lock）奇异情况
 */
static Eigen::Vector3d quaternion_to_euler(
    Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic = false) {
    if (!extrinsic) std::swap(axis0, axis2);

    auto i = axis0, j = axis1, k = axis2;
    auto is_proper = (i == k);
    if (is_proper) k = 3 - i - j;
    auto sign = (i - j) * (j - k) * (k - i) / 2;

    double a, b, c, d;
    Eigen::Vector4d xyzw = q.coeffs();
    if (is_proper) {
        a = xyzw[3];
        b = xyzw[i];
        c = xyzw[j];
        d = xyzw[k] * sign;
    } else {
        a = xyzw[3] - xyzw[j];
        b = xyzw[i] + xyzw[k] * sign;
        c = xyzw[j] + xyzw[3];
        d = xyzw[k] * sign - xyzw[i];
    }

    Eigen::Vector3d eulers;
    auto n2   = a * a + b * b + c * c + d * d;
    eulers[1] = std::acos(2 * (a * a + b * b) / n2 - 1);

    auto half_sum  = std::atan2(b, a);
    auto half_diff = std::atan2(-d, c);

    auto eps   = 1e-7;
    auto safe1 = std::abs(eulers[1]) >= eps;
    auto safe2 = std::abs(eulers[1] - std::numbers::pi) >= eps;
    auto safe  = safe1 && safe2;
    if (safe) {
        eulers[0] = half_sum + half_diff;
        eulers[2] = half_sum - half_diff;
    } else {
        if (!extrinsic) {
            eulers[0] = 0;
            if (!safe1) eulers[2] = 2 * half_sum;
            if (!safe2) eulers[2] = -2 * half_diff;
        } else {
            eulers[2] = 0;
            if (!safe1) eulers[0] = 2 * half_sum;
            if (!safe2) eulers[0] = 2 * half_diff;
        }
    }

    for (int i = 0; i < 3; i++)
        eulers[i] = clamp_pm_pi(eulers[i]);

    if (!is_proper) {
        eulers[2] *= sign;
        eulers[1] -= std::numbers::pi / 2;
    }

    if (!extrinsic) std::swap(eulers[0], eulers[2]);

    return eulers;
}

/**
 * @brief 将旋转矩阵转换为欧拉角
 * @param R 3×3 旋转矩阵
 * @param axis0 第一个旋转轴（0=X, 1=Y, 2=Z）
 * @param axis1 第二个旋转轴
 * @param axis2 第三个旋转轴
 * @param extrinsic 是否使用外旋（false=内旋）
 * @return 欧拉角向量（弧度）
 * @note 内部先转换为四元数，再调用 quaternion_to_euler
 */
static Eigen::Vector3d matrix_to_euler(
    Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic = false) {
    Eigen::Quaterniond q(R);
    return quaternion_to_euler(q, axis0, axis1, axis2, extrinsic);
}

/**
 * @brief 计算两个二维向量之间的夹角（绝对值）
 * @param vec1 第一个向量
 * @param vec2 第二个向量
 * @return 夹角（弧度），范围 [0, π]
 * @note 使用点积公式：θ = arccos(v1·v2 / (|v1||v2|))
 */
static inline double get_abs_angle(const Eigen::Vector2d& vec1, const Eigen::Vector2d& vec2) {
    if (vec1.norm() == 0. || vec2.norm() == 0.) {
        return 0.;
    }
    return std::acos(vec1.dot(vec2) / (vec1.norm() * vec2.norm())); // 范围 (0~π)
}

/**
 * @brief 计算两个四元数之间的偏航角误差
 * @param q1 第一个四元数
 * @param q2 第二个四元数
 * @return 偏航角误差（弧度），范围 [0, π]
 * @note 仅比较绕 Z 轴的旋转差异
 */
static inline double get_angle_err_rad_from_quaternion(
    const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
    double yaw1  = get_yaw_from_quaternion(q1);
    double yaw2  = get_yaw_from_quaternion(q2);
    auto yaw_err = abs(yaw1 - yaw2);

    while (yaw_err > 2 * std::numbers::pi)
        yaw_err -= 2 * std::numbers::pi;
    if (yaw_err > std::numbers::pi) yaw_err = 2 * std::numbers::pi - yaw_err;
    return yaw_err;
}

/**
 * @brief 计算两个三维向量的模长差异
 * @param v1 第一个向量
 * @param v2 第二个向量
 * @return 模长差的绝对值
 */
static inline double get_distance_err_rad_from_vector3d(
    const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
    double d1 = v1.norm();
    double d2 = v2.norm();
    auto derr = abs(d1 - d2);

    return derr;
}

/**
 * @brief 将笛卡尔坐标 (x, y, z) 转换为球坐标 (yaw, pitch, distance)
 * @param xyz 笛卡尔坐标 [x, y, z]
 * @return 球坐标 [yaw, pitch, distance]
 *         - yaw: 方位角（弧度），范围 [-π, π]
 *         - pitch: 俯仰角（弧度），范围 [-π/2, π/2]
 *         - distance: 距离（与原点的距离）
 * @note 常用于目标位置表示（从相机坐标系到云台角度）
 */
static inline Eigen::Vector3d xyz2ypd(const Eigen::Vector3d& xyz) {
    auto x = xyz[0], y = xyz[1], z = xyz[2];
    auto yaw      = std::atan2(y, x);
    auto pitch    = std::atan2(z, std::sqrt(x * x + y * y));
    auto distance = std::sqrt(x * x + y * y + z * z);
    return { yaw, pitch, distance };
}

/**
 * @brief 计算 xyz2ypd 函数的雅可比矩阵
 * @param xyz 笛卡尔坐标 [x, y, z]
 * @return 3×3 雅可比矩阵 J，其中 J_ij = ∂(ypd)_i / ∂(xyz)_j
 * @note 用于 EKF 中将笛卡尔坐标系的协方差传播到球坐标系
 * @details 雅可比矩阵：
 *          J = [ ∂yaw/∂x,      ∂yaw/∂y,      ∂yaw/∂z      ]
 *              [ ∂pitch/∂x,    ∂pitch/∂y,    ∂pitch/∂z    ]
 *              [ ∂distance/∂x, ∂distance/∂y, ∂distance/∂z ]
 */
static Eigen::Matrix<double, 3, 3> xyz2ypd_jacobian(const Eigen::Vector3d& xyz) {
    auto x = xyz[0], y = xyz[1], z = xyz[2];

    auto dyaw_dx = -y / (x * x + y * y);
    auto dyaw_dy = x / (x * x + y * y);
    auto dyaw_dz = 0.0;

    auto dpitch_dx = -(x * z) / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 1.5));
    auto dpitch_dy = -(y * z) / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 1.5));
    auto dpitch_dz = 1 / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 0.5));

    auto ddistance_dx = x / std::pow((x * x + y * y + z * z), 0.5);
    auto ddistance_dy = y / std::pow((x * x + y * y + z * z), 0.5);
    auto ddistance_dz = z / std::pow((x * x + y * y + z * z), 0.5);

    // clang-format off
  Eigen::Matrix<double,3,3> J{
    {dyaw_dx, dyaw_dy, dyaw_dz},
    {dpitch_dx, dpitch_dy, dpitch_dz},
    {ddistance_dx, ddistance_dy, ddistance_dz}
  };
    // clang-format on

    return J;
}

/**
 * @brief 计算数值的平方
 * @tparam T 数值类型（支持乘法运算）
 * @param a 输入值
 * @return a²
 */
template <typename T> T square(T const& a) { return a * a; };

} // namespace world_exe::util::math
