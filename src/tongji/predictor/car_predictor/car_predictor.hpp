/**
 * @file src/tongji/predictor/car_predictor/car_predictor.hpp
 * @brief Predictor component for Car Predictor.
 */

#pragma once

#include <cstdlib>
#include <ctime>
#include <memory>
#include <optional>

#include <Eigen/Dense>

#include "../in_gimbal_control_armor.hpp"
#include "../kalman_filter/extended_kalman_filter.hpp"
#include "../kalman_filter/predict_model.hpp"
#include "data/armor_gimbal_control_spacing.hpp"
#include "data/time_stamped.hpp"
#include "enum/car_id.hpp"
#include "interfaces/predictor.hpp"

namespace world_exe::tongji::predictor {

/**
 * @brief 基于 EKF 的整车预测器，输出未来装甲板轨迹。
 */
class CarPredictor final : public interfaces::IPredictor {
public:
    using PredictorModel = EKFModel<11, 4>;
    using EKF            = ExtendedKalmanFilter<PredictorModel>;

    /**
     * @brief 使用现有 EKF 状态构造快照。
     */
    explicit CarPredictor(
        const EKF& ekf, const PredictorModel& model, const data::TimeStamp& time_stamp)
        : ekf_(ekf)
        , model_(model)
        , time_stamp_(time_stamp) { }

    /**
     * @brief 依据单帧装甲观测初始化预测器。
     *
     * @param armor_xyz_in_gimbal 云台坐标系位姿
     * @param armor_ypr_in_gimbal 欧拉角
     * @param car_id 车辆 ID
     * @param time_stamp 观测时间
     */
    explicit CarPredictor(const Eigen::Vector3d& armor_xyz_in_gimbal,
        const Eigen::Vector3d& armor_ypr_in_gimbal, const enumeration::CarIDFlag& car_id,
        const data::TimeStamp& time_stamp)
        : time_stamp_(time_stamp)
        , model_(car_id)
        , car_id_(car_id) {
        // x vx y vy z vz a w r l h
        // a: angle
        // w: angular velocity
        // l: r2 - r1
        // h: z2 - z1
        auto center_x =
            armor_xyz_in_gimbal[0] + model_.GetRadius() * std::cos(armor_ypr_in_gimbal[0]);
        auto center_y =
            armor_xyz_in_gimbal[1] + model_.GetRadius() * std::sin(armor_ypr_in_gimbal[0]);
        auto center_z = armor_xyz_in_gimbal[2];

        EKF::XVec x0 { center_x, 0, center_y, 0, center_z, 0, armor_ypr_in_gimbal[0], 0,
            model_.GetRadius(), 0, 0 };

        EKF::PMat P0 = model_.GetP0Dig().asDiagonal();
        ekf_.emplace(x0, P0, model_); // 初始化滤波器（预测量、预测量协方差）

    }

    const enumeration ::ArmorIdFlag& GetId() const override { return car_id_; }

    /**
     * @brief 预测指定时间的装甲板集合。
     */
    std ::shared_ptr<interfaces::IArmorInGimbalControl> Predictor(
        const data ::TimeStamp& time_stamp) const override {
        const auto ekf_x = this->GetPredictedX((time_stamp - time_stamp_).to_seconds());
        std::vector<data::ArmorGimbalControlSpacing> armors;
        for (int id = 0; id < model_.GetArmorNum(); id++) {
            auto angle = util::math::clamp_pm_pi(ekf_x[6] + id * 2 * CV_PI / model_.GetArmorNum());
            auto xyz   = model_.h_armor_xyz(ekf_x, id);

            data::ArmorGimbalControlSpacing armor;
            armor.id          = model_.GetID();
            armor.position    = xyz;
            armor.orientation = util::math::euler_to_quaternion(angle, 15. / 180. * CV_PI, 0);
            armors.emplace_back(std::move(armor));
        }
        return std::make_shared<InGimbalControlArmor>(armors, time_stamp_);
    }

    /// 返回当前 EKF 状态向量。
    EKF::XVec GetEkfX() const { return ekf_->x; }
    /// 返回内部使用的预测模型。
    auto GetModel() const -> const PredictorModel { return model_; }
    /// 返回 EKF 实例。
    auto GetEkf() const -> const EKF { return ekf_.value(); }

    /// 最近一次更新的时间戳。
    data::TimeStamp LastSeen() const { return time_stamp_; }

    /// 获取未来 dt 秒时各装甲板的 XYZA。
    auto GetPredictedXYZAList(const double& dt) -> std::vector<Eigen::Vector4d> const {
        const auto [x_n, P_n] = ekf_->PredictOnce(dt);
        return model_.GetArmorXYZAList(x_n);
    }

    auto GetPredictedX(const double& dt) const -> const EKF::XVec {
        const auto& [x_n, P_n] = ekf_->PredictOnce(dt);
        return x_n;
    }

    /**
     * @brief 使用新观测更新状态。
     */
    void Update(const data::TimeStamp time_stamp, const Eigen::Vector3d& armor_xyz_in_gimbal,
        const Eigen::Vector3d& armor_ypr_in_gimbal, const Eigen::Vector3d& armor_ypd_in_gimbal) {

        // 装甲板匹配
        int id = model_.MatchArmor(
            ekf_->x, armor_xyz_in_gimbal, armor_ypr_in_gimbal, armor_ypd_in_gimbal);
        last_id_ = id;
        update_count_++;

        Update_ypda(armor_xyz_in_gimbal, armor_ypr_in_gimbal, armor_ypd_in_gimbal, id,
            (time_stamp - time_stamp_).to_seconds());

        time_stamp_ = time_stamp;
    }

    /// 判断 EKF 是否已收敛到合理半径/长度。
    bool IsConverged() const {
        auto r_ok = ekf_->x[8] > 0.05 && ekf_->x[8] < 0.5;
        auto l_ok = ekf_->x[8] + ekf_->x[9] > 0.05 && ekf_->x[8] + ekf_->x[9] < 0.5;
        auto rot  = abs(ekf_->x[7]) > 0.3f;

        if (r_ok && l_ok && rot) return false;
        // util::logger::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_->x[8], ekf_->x[9]);
        return true;
    }
    /// 判断目标是否已出现足够次数。
    auto IsAppeared() -> bool {
        const int required_count = (model_.GetID() == enumeration::CarIDFlag::Outpost) ? 10 : 3;
        return update_count_ > required_count;
    }

private:
    void Update_ypda(const Eigen::Vector3d& armor_xyz_in_gimbal,
        const Eigen::Vector3d& armor_ypr_in_gimbal, const Eigen::Vector3d& armor_ypd_in_gimbal,
        const int& id, const double& dt) {
        // 观测jacobi
        auto H = model_.H(ekf_->x, id);
        auto R = model_.R(armor_xyz_in_gimbal, armor_ypr_in_gimbal, armor_ypd_in_gimbal, id);

        const Eigen::Vector3d& ypd = armor_ypd_in_gimbal;
        const Eigen::Vector3d& ypr = armor_ypr_in_gimbal;

        // 获得观测量
        EKF::ZVec z(4);
        z << ypd[0], ypd[1], ypd[2], ypr[0];

        ekf_->Update(dt, z, H, R, id);

        // 前哨站转速特判
        if (model_.GetID() == enumeration::CarIDFlag::Outpost) {
            constexpr double max_outpost_w = 2.51;
            if (std::abs(ekf_->x[7]) > 2.0) {
                ekf_->x[7] = ekf_->x[7] > 0 ? max_outpost_w : -max_outpost_w;
            }
        }
    }

    data::TimeStamp time_stamp_;
    PredictorModel model_;
    std::optional<EKF> ekf_;
    enumeration::CarIDFlag car_id_;

    int last_id_                           = -1;
    int update_count_                      = 0;
    const double max_allowed_failure_rate_ = 0.4;
};
}
