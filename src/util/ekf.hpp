/**
 * @file src/util/ekf.hpp
 * @brief Utility helper for Ekf.
 */

#pragma once

#include "eigen3/Eigen/Eigen"

namespace world_exe::util {
/**
 * @brief 通用扩展卡尔曼滤波器（EKF）基类模板
 *
 * @tparam xn 状态向量维度
 * @tparam zn 观测向量维度
 * @tparam IEkf 派生类类型（CRTP模式），需实现 f, h, A, W, H, V, Q, R 等函数
 *
 * @note 使用 CRTP (Curiously Recurring Template Pattern) 实现静态多态
 * @note 派生类必须实现以下虚函数：
 *       - process_z(z_k): 预处理观测值（可选，默认不处理）
 *       - normalize_x(X_k): 归一化状态向量（可选，默认不处理）
 *       - f(x, u, w, dt): 状态转移函数
 *       - h(x, v): 观测函数
 *       - A(x, u, w, dt): 状态转移雅可比矩阵 ∂f/∂x
 *       - W(x, u, w): 过程噪声雅可比矩阵 ∂f/∂w
 *       - H(x, v): 观测雅可比矩阵 ∂h/∂x
 *       - V(x, v): 观测噪声雅可比矩阵 ∂h/∂v
 *       - Q(dt): 过程噪声协方差矩阵
 *       - R(z): 观测噪声协方差矩阵
 */
template <int xn, int zn, typename IEkf> class Ekf {
public:
    typedef Eigen::Matrix<double, xn, 1> XVec;  ///< 状态向量类型
    typedef XVec UVec;                          ///< 控制输入向量类型
    typedef XVec WVec;                          ///< 过程噪声向量类型

    typedef Eigen::Matrix<double, zn, 1> ZVec;  ///< 观测向量类型
    typedef ZVec VVec;                          ///< 观测噪声向量类型

    typedef Eigen::Matrix<double, xn, xn> PMat; ///< 状态协方差矩阵类型
    typedef Eigen::Matrix<double, zn, zn> RMat; ///< 观测噪声协方差矩阵类型
    typedef Eigen::Matrix<double, xn, xn> AMat; ///< 状态转移雅可比矩阵类型
    typedef Eigen::Matrix<double, xn, xn> WMat; ///< 过程噪声雅可比矩阵类型
    typedef Eigen::Matrix<double, zn, zn> VMat; ///< 观测噪声雅可比矩阵类型
    typedef Eigen::Matrix<double, xn, xn> QMat; ///< 过程噪声协方差矩阵类型
    typedef Eigen::Matrix<double, zn, xn> HMat; ///< 观测雅可比矩阵类型
    typedef Eigen::Matrix<double, xn, zn> KMat; ///< 卡尔曼增益矩阵类型

    /**
     * @brief 获取当前状态估计值
     * @return 当前状态向量 X_k
     */
    [[nodiscard]] inline XVec OutPut() const { return X_k; }

    /**
     * @brief 执行EKF更新步骤（预测 + 校正）
     *
     * @param z_k 当前时刻的观测值
     * @param u_k 当前时刻的控制输入
     * @param dt 时间步长（秒）
     *
     * @note EKF标准更新流程：
     *       1. 预测步骤：
     *          - x̂_k|k-1 = f(x̂_k-1, u_k, 0, dt)
     *          - P_k|k-1 = A_k * P_k-1 * A_k^T + W_k * Q_k * W_k^T
     *       2. 校正步骤：
     *          - ỹ_k = z_k - h(x̂_k|k-1, 0)              (创新/残差)
     *          - S_k = H_k * P_k|k-1 * H_k^T + V_k * R_k * V_k^T  (创新协方差)
     *          - K_k = P_k|k-1 * H_k^T * S_k^-1         (卡尔曼增益)
     *          - x̂_k = x̂_k|k-1 + K_k * ỹ_k             (状态更新)
     *          - P_k = (I - K_k * H_k) * P_k|k-1        (协方差更新)
     */
    inline void Update(const ZVec& z_k, const UVec& u_k, const double& dt) {
        dt_ = dt;
        // 初始化临时变量
        P_k_n.setZero();
        S_k.setZero();
        y_k.setZero();
        K_t.setZero();
        tmpK.setZero();

        auto derived = static_cast<IEkf*>(this);

        // 预处理观测值（派生类可自定义）
        const auto processed_z = derived->process_z(z_k);

        // 计算雅可比矩阵（在当前状态点线性化）
        auto x_k_n = derived->f(X_k, u_k, w_zero, dt);  // 状态预测
        auto A_k   = derived->A(X_k, u_k, w_zero, dt);  // ∂f/∂x
        auto W_k   = derived->W(X_k, u_k, w_zero);      // ∂f/∂w
        auto H_k   = derived->H(x_k_n, v_zero);         // ∂h/∂x
        auto V_k   = derived->V(x_k_n, v_zero);         // ∂h/∂v

        // 预测协方差：P_k|k-1 = A * P * A^T + W * Q * W^T
        P_k_n = A_k * P_k * A_k.transpose() + W_k * derived->Q(dt) * W_k.transpose();

        // 计算创新（残差）：y = z - h(x̂)
        y_k  = processed_z - derived->h(x_k_n, v_zero);
        // 创新协方差：S = H * P * H^T + V * R * V^T
        S_k  = H_k * P_k_n * H_k.transpose() + V_k * derived->R(processed_z) * V_k.transpose();
        // 卡尔曼增益：K = P * H^T * S^-1
        K_t  = P_k_n * H_k.transpose() * S_k.inverse();
        // 状态更新：x̂ = x̂_pred + K * y
        X_k  = x_k_n + K_t * y_k;
        // 归一化状态向量（派生类可自定义，如角度归一化）
        X_k  = derived->normalize_x(X_k);
        // 协方差更新：P = (I - K * H) * P_pred
        tmpK = Eye_K - K_t * H_k;
        P_k  = tmpK * P_k_n;
    }

    /**
     * @brief 派生类需要实现以下函数（CRTP接口）
     *
     * @details 可选函数（有默认实现）：
     *          - ZVec process_z(const ZVec& z_k): 预处理观测值，默认直接返回
     *          - XVec normalize_x(const XVec& X_k): 归一化状态，默认直接返回
     *
     * @details 必须实现的纯虚函数：
     *          - XVec f(x, u, w, dt): 非线性状态转移函数 x_k = f(x_{k-1}, u_k, w_k, dt)
     *          - ZVec h(x, v): 非线性观测函数 z_k = h(x_k, v_k)
     *          - AMat A(x, u, w, dt): 状态转移雅可比矩阵 A = ∂f/∂x
     *          - WMat W(x, u, w): 过程噪声雅可比矩阵 W = ∂f/∂w
     *          - HMat H(x, v): 观测雅可比矩阵 H = ∂h/∂x
     *          - VMat V(x, v): 观测噪声雅可比矩阵 V = ∂h/∂v
     *          - QMat Q(dt): 过程噪声协方差矩阵（时变）
     *          - RMat R(z): 观测噪声协方差矩阵（可能依赖观测值）
     */
    // [[nodiscard]] virtual ZVec process_z(const ZVec& z_k) { return z_k; }
    // [[nodiscard]] virtual XVec normalize_x(const XVec& X_k) { return X_k; }
    // [[nodiscard]] virtual XVec f(const XVec&, const UVec&, const WVec&, const double&) = 0;
    // [[nodiscard]] virtual ZVec h(const XVec&, const VVec&)                             = 0;
    //
    // [[nodiscard]] virtual AMat A(const XVec&, const UVec&, const WVec&, const double&) = 0;
    //
    // [[nodiscard]] virtual WMat W(const XVec&, const UVec&, const WVec&) = 0;
    //
    // [[nodiscard]] virtual HMat H(const XVec&, const VVec&) = 0;
    //
    // [[nodiscard]] virtual VMat V(const XVec&, const VVec&) = 0;
    //
    // [[nodiscard]] virtual QMat Q(const double& t) = 0;
    // [[nodiscard]] virtual RMat R(const ZVec& z)   = 0;

protected:
    XVec X_k { XVec::Zero() };      ///< 当前状态估计值 x̂_k
    PMat P_k { PMat::Identity() };  ///< 当前状态协方差矩阵 P_k

    double dt_;  ///< 上次更新的时间步长（秒）

private:
    PMat P_k_n {};  ///< 预测协方差矩阵 P_k|k-1（临时变量）
    RMat S_k {};    ///< 创新协方差矩阵 S_k（临时变量）
    ZVec y_k {};    ///< 创新向量（残差）ỹ_k = z_k - h(x̂_k|k-1)
    KMat K_t {};    ///< 卡尔曼增益矩阵 K_k
    PMat tmpK {};   ///< 临时矩阵，用于计算 (I - K*H)

    static inline const WVec w_zero = Eigen::Matrix<double, xn, 1>::Zero();      ///< 零过程噪声向量
    static inline const VVec v_zero = Eigen::Matrix<double, zn, 1>::Zero();      ///< 零观测噪声向量
    static inline const PMat Eye_K  = Eigen::Matrix<double, xn, xn>::Identity(); ///< 单位矩阵
};
}
