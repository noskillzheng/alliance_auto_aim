/**
 * @file src/tongji/predictor/kalman_filter/extended_kalman_filter.hpp
 * @brief Predictor component for Extended Kalman Filter.
 */

#pragma once

#include <concepts>
#include <deque>
#include <numeric>

#include <Eigen/Dense>

namespace world_exe::tongji::predictor {
/**
 * @brief 要求模型类型提供 EKF 所需的别名与静态成员。
 */
template <typename T>
concept EKFModelTypes = requires {
    { T::xn } -> std::same_as<const int&>;
    { T::zn } -> std::same_as<const int&>;

    typename T::XVec;
    typename T::ZVec;
    typename T::AMat;
    typename T::PMat;
    typename T::RMat;
    typename T::QMat;
    typename T::HMat;
};

/**
 * @brief 通用扩展卡尔曼滤波器实现。
 */
template <typename EKFModel> class ExtendedKalmanFilter {
public:
    static constexpr int xn = EKFModel::xn;
    static constexpr int zn = EKFModel::zn;

    using XVec = EKFModel::XVec;
    using ZVec = EKFModel::ZVec;
    using AMat = EKFModel::AMat;
    using PMat = EKFModel::PMat;
    using PDig = EKFModel::PDig;
    using RMat = EKFModel::RMat;
    using RDig = EKFModel::RDig;
    using QMat = EKFModel::QMat;
    using HMat = EKFModel::HMat;

    XVec x;
    PMat P;

    /**
     * @brief 以初始状态和协方差构造滤波器。
     */
    ExtendedKalmanFilter(const XVec& x0, const PMat& P0, const EKFModel& model)
        : x(x0)
        , P(P0)
        , model_(model)
        , I(Eigen::Matrix<double,xn,xn>::Identity()) {
        // data["residual_yaw"]        = 0.0;
        // data["residual_pitch"]      = 0.0;
        // data["residual_distance"]   = 0.0;
        // data["residual_angle"]      = 0.0;
        // data["nis"]                 = 0.0;
        // data["nees"]                = 0.0;
        // data["nis_fail"]            = 0.0;
        // data["nees_fail"]           = 0.0;
        // data["recent_nis_failures"] = 0.0;
    }

    // 无副作用，不修改x，仅预测
    /// 仅进行一次预测，不修改内部状态。
    auto PredictOnce(const double& dt) const -> const std::pair<XVec, PMat> {
        const auto A   = model_.A(dt);
        const auto Q   = model_.Q(dt);
        const auto x_n = model_.f(x, dt);
        const auto P_n = A * P * A.transpose() + Q;

        return { x_n, P_n };
    }

    /**
     * @brief 使用新观测更新状态。
     */
    auto Update(const double& dt, const ZVec& z, const HMat& H, const RMat& R, const int& id)
        -> const XVec {
        const auto [x_prior, P_prior] = PredictOnce(dt);
        const ZVec z_prior            = model_.h(x_prior, id);

        const ZVec residual = model_.z_subtract(z, z_prior);

        const RMat S = H * P_prior * H.transpose() + R;
        const Eigen::MatrixXd K = P_prior * H.transpose() * S.inverse();
        // std::cout << P << "\n" << std::endl;

        x << model_.x_add(x_prior, K * residual);

        // Stable Compution of the Posterior Covariance
        // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
        // FIXME: P -> P^-
        // P << (I - K * H) * P_prior * (I - K * H).transpose() + K * R * K.transpose();
        P << (I - K * H) * P_prior;
        const auto P_inverse = P_prior.inverse();
        const auto error     = x - x_prior;

        RecordStatistics(residual, S, error, P_inverse);

        return x;
    }

    /// 判断 NIS 失败率是否超过阈值。
    auto IsDiverse() const -> bool const { return CalculateFailureRate() >= max_failure_rate; }

private:
    auto CalculateFailureRate() const -> bool const {
        int failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
        return (double)failures / window_size;
    }

    auto GetRecentNISFailureRate() const -> double const {
        if (recent_nis_failures.empty()) return 0.0;

        int recent_failures =
            std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
        return (double)recent_failures / recent_nis_failures.size();
    }

    auto RecordStatistics(
        const ZVec& residual, const RMat& S, const XVec& error, const PMat& P_inverse) -> void {

        /// 卡方检验
        // 新增检验
        double nis  = residual.transpose() * S.inverse() * residual;
        double nees = (error).transpose() * P.inverse() * error;

        total_count_++;
        last_nis  = nis;
        last_nees = nees;

        if (nis > nis_threshold_) nis_count_++;
        if (nees > nees_threshold_) nees_count_++;
        recent_nis_failures.push_back(nis > nis_threshold_ ? 1 : 0);
        if (recent_nis_failures.size() > window_size) {
            recent_nis_failures.pop_front();
        }
    }

    const Eigen::Matrix<double, xn, xn> I;  ///< 单位矩阵，用于协方差更新
    const EKFModel& model_;                  ///< EKF 模型引用

    /**
     * @brief NIS (Normalized Innovation Squared) 卡方检验阈值
     * @note 自由度 = zn（观测维度），置信水平 95%
     * @note 查卡方分布表：χ²(zn, 0.95)，此处 zn=4 时约为 0.711
     * @note 用于检测滤波器是否发散：NIS = y^T * S^-1 * y
     * @note 若 NIS > threshold，表示观测残差异常大，可能滤波器发散
     */
    const double nis_threshold_ = 0.711;

    /**
     * @brief NEES (Normalized Estimation Error Squared) 卡方检验阈值
     * @note 自由度 = xn（状态维度），置信水平 95%
     * @note 用于检测状态估计误差是否与协方差矩阵一致
     * @note NEES = (x - x_prior)^T * P^-1 * (x - x_prior)
     * @note 若 NEES > threshold，表示估计不一致
     */
    const double nees_threshold_ = 0.711;

    /**
     * @brief 最大 NIS 失败率阈值
     * @note 在滑动窗口内，NIS 超阈值的比例 > 40% 时判定滤波器发散
     * @note 40% 的设定基于经验值：理论上 95% 置信水平应有 5% 失败率，
     *       但实际中允许一定容错，40% 是明显异常的界限
     */
    const double max_failure_rate = 0.4;

    // std::map<std::string, double> data; // 卡方检验数据（已注释）
    std::deque<int> recent_nis_failures { 0 };  ///< 最近 NIS 失败记录（滑动窗口）
    size_t window_size = 100;                    ///< 滑动窗口大小（样本数）
    double last_nis;                             ///< 上次 NIS 值
    double last_nees;                            ///< 上次 NEES 值
    int nees_count_  = 0;                        ///< NEES 累计失败次数
    int nis_count_   = 0;                        ///< NIS 累计失败次数
    int total_count_ = 0;                        ///< 总更新次数
};

} // namespace tools
