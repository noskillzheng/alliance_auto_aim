# Alliance Auto Aim 代码对比报告

**远程仓库**: https://github.com/noskillzheng/RMCS-Flight/tree/playground/rmcs_ws/src/alliance_ros_auto_aim/alliance_auto_aim

**本地目录**: RMCS/rmcs_ws/src/alliance_ros_auto_aim/alliance_auto_aim

**对比时间**: 2025-12-02

---

## 1. 构建配置变更

### CMakeLists.txt

```diff
-set(CMAKE_CXX_STANDARD 23)
+set(CMAKE_CXX_STANDARD 20)
```

**变更说明**: C++标准从C++23降级到C++20

---

## 2. 配置文件变更

### configs/example.yaml

```diff
+#####-----fire_controller parameters-----#####
+control_delay_s: 0.1

 #####-----aiming_solver parameters-----#####
 yaw_offset: 1.5 # degree
 pitch_offset: -0.5 # degree
-bullet_speed: 20
+bullet_speed: 26

 #####-----solver parameters-----#####
-# R_muzzle2camera:
-# t_muzzle2camera:
+R_muzzle2camera:
+t_muzzle2camera:
```

**变更说明**:
- 新增 `control_delay_s` 参数
- 子弹速度从20改为26
- 取消注释 `R_muzzle2camera` 和 `t_muzzle2camera`

---

## 3. 核心接口变更

### include/interfaces/identifier.hpp

```diff
 virtual const std::tuple<const std::shared_ptr<IArmorInImage>, enumeration::CarIDFlag> identify(
-    const cv::Mat& input_image, const data::TimeStamp& timestamp) = 0;
+    const cv::Mat& input_image) = 0;
```

**变更说明**: `identify()` 接口移除了 `timestamp` 参数

### include/interfaces/fire_controller.hpp

```diff
-virtual data::FireControl CalculateTarget(data::TimeStamp const& time_stamp) const = 0;
+virtual const data::FireControl CalculateTarget(const std::chrono::seconds& time_duration) const = 0;

-virtual enumeration::CarIDFlag GetAttackCarId() const = 0;
+virtual const enumeration::CarIDFlag GetAttackCarId() const = 0;
```

**变更说明**:
- `CalculateTarget()` 参数从 `TimeStamp` 改为 `std::chrono::seconds`
- 返回值添加 `const` 限定符

### include/interfaces/predictor.hpp

```diff
-// virtual data::TimeStamp GetTimeStamp() const = 0;
```

**变更说明**: 移除了注释掉的 `GetTimeStamp()` 接口声明

---

## 4. 数据结构变更

### include/data/fire_control.hpp

```diff
-Eigen::Vector3d gimbal_dir = Eigen::Vector3d::Zero();
+Eigen::Vector3d gimbal_dir;
```

**变更说明**: `gimbal_dir` 移除了默认初始化

### include/data/mat_stamped.hpp

```diff
-MatStamped(cv::Mat&& image, data::TimeStamp ts)
+MatStamped(cv::Mat&& image, data::TimeStamp stamp)
 {
-    mat = std::move(image);
-    stamp = ts;
+    mat = image;
+    stamp = stamp;
 }

 void Load(const cv::Mat& image, data::TimeStamp now)
 {
-    // 防御性检查：如果 mat 已存在但类型不兼容，先释放避免 copyTo 断言失败
-    if (!mat.empty() && mat.type() != image.type()) {
-        mat.release();
-    }
     image.copyTo(mat);
     stamp = now;
 }
```

**变更说明**:
- 移除了 `std::move` (可能导致拷贝而非移动)
- 移除了 `Load()` 中的防御性类型检查

### include/data/time_stamped.hpp

```diff
-inline operator std::chrono::steady_clock::time_point() const {
-    return std::chrono::steady_clock::time_point{stamp_};
-}
```

**变更说明**: 移除了到 `steady_clock::time_point` 的隐式转换运算符

---

## 5. 事件总线变更

### include/core/event_bus.hpp

```diff
 template <typename TData>
-static size_t Subscript(const std::string& event_name, std::function<void(const TData&)>&& func) {
+static void Subscript(const std::string& event_name, std::function<void(const TData&)>&& func) {
     return EventBusImpl<TData>::GetInstance().Subscript(event_name, std::move(func));
 }

-template <typename TData>
-static void Unsubscribe(const std::string& event_name, size_t subscription_id) {
-    return EventBusImpl<TData>::GetInstance().Unsubscribe(event_name, subscription_id);
-}
```

**变更说明**:
- `Subscript()` 不再返回订阅ID
- 移除了 `Unsubscribe()` 取消订阅功能

---

## 6. 参数系统变更

### include/parameters/params_system_v1.hpp

```diff
-static void set_target_color(bool target_color);  // true=RED, false=BLUE
-static bool target_color();                       // true=RED, false=BLUE
```

### src/util/parameters/params_system_v1.cpp

```diff
-std::string model_path =
-    std::filesystem::path { __FILE__ }.parent_path().parent_path().parent_path().parent_path()
-    / "models" / "szu_identify_model.onnx";
+std::string model_path = "/workspaces/src/alliance_ros_auto_aim/alliance_auto_aim/models/"
+                         "szu_identify_model.onnx";

-bool target_color_ = false;  // false=BLUE, true=RED

-void ParamsForSystemV1::set_target_color(bool target_color) { ... }
-bool ParamsForSystemV1::target_color() { ... }
```

**变更说明**:
- 模型路径从相对路径改为硬编码绝对路径
- 移除了 `target_color` 参数的getter/setter

---

## 7. 自瞄系统主流程变更

### src/tongji/auto_aim_system.cpp

```diff
-explicit Impl(const bool& debug)
+Impl(const bool& debug)
     : debug(debug)
-    , config_path_(std::filesystem::path { __FILE__ }.parent_path().parent_path().parent_path()
-          / "configs"
-          / "example.yaml")
+    , config_path_("/workspaces/src/alliance_ros_auto_aim/alliance_auto_aim/configs/example.yaml")

-const auto& [armors_in_image, flag] = identifier_->identify(raw.mat, raw.stamp);
+const auto& [armors_in_image, flag] = identifier_->identify(raw.mat);

-state_machine_->Update(armors_in_image, enumeration::CarIDFlag::None, time_stamp_);
+state_machine_->Update(armors_in_image, enumeration::CarIDFlag::None,
+    std::chrono::duration_cast<milliseconds>(
+        std::chrono::steady_clock::now() - time_stamp_));

-data::TimeStamp time_stamp_;
+std::chrono::steady_clock::time_point time_stamp_;

-return fire_controller_->CalculateTarget(
-    data::TimeStamp(steady_clock::now().time_since_epoch()));
+return fire_controller_->CalculateTarget(
+    std::chrono::duration_cast<seconds>(std::chrono::steady_clock::now() - time_stamp_));
```

**变更说明**:
- 配置路径从相对路径改为硬编码绝对路径
- 移除了 `explicit` 构造函数修饰符
- `identify()` 调用移除了时间戳参数
- 时间戳类型从自定义 `TimeStamp` 改为 `steady_clock::time_point`
- `CalculateTarget()` 参数从绝对时间戳改为相对时间差

---

## 8. 火控系统变更

### src/tongji/fire_controller/fire_controller.cpp

```diff
-data ::FireControl CalculateTarget(data::TimeStamp const& time_stamp) const {
+const data ::FireControl CalculateTarget(
+    const std::chrono::seconds& time_from_tracker_timepoint) const {

-const auto& aim_solution =
-    aiming_solver_->SolveAimSolution(snapshot_manager, time_stamp, control_delay_);
+const auto& aim_solution = aiming_solver_->SolveAimSolution(
+    snapshot_manager, time_from_tracker_timepoint, control_delay_s_);

-result.gimbal_dir << cos(gimbal_command.yaw) * cos(gimbal_command.pitch),
-    sin(gimbal_command.yaw) * cos(gimbal_command.pitch), sin(gimbal_command.pitch);
+result.gimbal_dir << gimbal_command.yaw, gimbal_command.pitch, 0;

-result.fire_allowance = true;
+result.fire_allowance = fire_command;

-std::chrono::milliseconds control_delay_;
+double control_delay_s_;

-mutable bool firable_;
+mutable double firable_;
```

**变更说明**:
- 控制延迟从 `milliseconds` 改为 `double` 秒
- `gimbal_dir` 输出格式从方向向量改为欧拉角
- `fire_allowance` 从始终为true改为根据fire_command判断
- `firable_` 类型从 `bool` 改为 `double` (可能是bug)

### src/tongji/fire_controller/aim_solver.hpp

```diff
-AimSolution SolveAimSolution(std::shared_ptr<interfaces::IPredictor> const& snapshot,
-    data::TimeStamp const& time_stamp, std::chrono::milliseconds control_delay) {
+AimSolution SolveAimSolution(std::shared_ptr<interfaces::IPredictor> snapshot,
+    data::TimeStamp time_stamp, const double& control_delay_s) {

-const auto& dt = prev_fly_time_s + (double)(control_delay).count() / 1000.;
+const auto& dt = control_delay_s + prev_fly_time_s;

-const auto& aim_point = SelectPredictedAim(
-    snapshot_derived->GetPredictedX(time_stamp), armors->GetArmors(snapshot->GetId()), snapshot->GetId());
+const auto& aim_point = SelectPredictedAim(snapshot_derived->GetPredictedX(dt),
+    armors->GetArmors(snapshot->GetId()), snapshot->GetId());

-const double pitch = (final_trajectory.pitch + pitch_offset_);
+const double pitch = -(final_trajectory.pitch + pitch_offset_);
```

**变更说明**:
- 控制延迟参数类型从 `milliseconds` 改为 `double`
- `GetPredictedX()` 参数从 `TimeStamp` 改为 `double` dt
- pitch 输出添加了负号

---

## 9. 识别器变更

### src/tongji/identifier/identifier.cpp

```diff
-std::tuple<const std::shared_ptr<interfaces::IArmorInImage>, enumeration::CarIDFlag> Identify(
-    const cv::Mat& bgr_img, const data::TimeStamp& timestamp) {
-    static_cast<void>(timestamp);
+const std::tuple<const std::shared_ptr<interfaces::IArmorInImage>, enumeration::CarIDFlag>
+Identify(const cv::Mat& bgr_img) {

-static Color GetColor(const cv::Mat& bgr_img, const std::vector<cv::Point>& contour) {
+Color GetColor(const cv::Mat& bgr_img, const std::vector<cv::Point>& contour) const {

-static cv::Mat GetPattern(...) {
+cv::Mat GetPattern(...) const {
```

**变更说明**:
- `Identify()` 移除了时间戳参数
- 多个静态方法改为成员方法

### src/tongji/identifier/tracker.hpp

```diff
 auto SelectTrackingTargetID(const std::shared_ptr<interfaces::IArmorInImage>& armors_in_image,
-    const enumeration::CarIDFlag& invincible_armors, data::TimeStamp const& time_stamp) noexcept
+    const enumeration::CarIDFlag& invincible_armors,
+    const std::chrono::milliseconds& duration_from_last_update) noexcept

-void CheckCameraOffline(data::TimeStamp const& time_stamp) {
-    auto duration_from_last_update = time_stamp - time_stamp_;
-    if ((duration_from_last_update.to_nanosec()
-            > (double)std::chrono::duration_cast<std::chrono::nanoseconds>(timeout_).count())) {
+void CheckCameraOffline(const std::chrono::milliseconds duration_from_last_update) {
+    if ((duration_from_last_update > timeout_)) {

-data::TimeStamp time_stamp_;
```

**变更说明**:
- 时间参数从绝对时间戳改为相对时间差
- 移除了内部时间戳存储
- 超时检查逻辑简化

---

## 10. 预测器变更

### src/tongji/predictor/car_predictor/car_predictor.hpp

```diff
-explicit CarPredictor(const EKF& ekf, PredictorModel model, const data::TimeStamp& time_stamp)
+explicit CarPredictor(
+    const EKF& ekf, const PredictorModel& model, const data::TimeStamp& time_stamp)
     : ekf_(ekf)
-    , model_(std::move(model))
+    , model_(model)

-return std::make_shared<InGimbalControlArmor>(armors, time_stamp);
+return std::make_shared<InGimbalControlArmor>(armors, time_stamp_);

-auto GetPredictedX(data::TimeStamp const& time_stamp) -> EKF::XVec {
-    auto dt = (time_stamp - time_stamp_).to_seconds();
-    return GetPredictedX(dt);
-}

+data::TimeStamp LastSeen() const { return time_stamp_; }

-void Update(data::TimeStamp const& time_stamp, ...) {
+void Update(const data::TimeStamp time_stamp, ...) {
```

**变更说明**:
- `model` 参数改为 `const&` (移除了move语义)
- `Predictor()` 返回的时间戳使用内部存储而非传入值
- 移除了接受 `TimeStamp` 的 `GetPredictedX()` 重载
- 新增 `LastSeen()` 方法

### src/tongji/predictor/kalman_filter/predict_model.hpp

```diff
-explicit EKFModel(const enumeration::CarIDFlag& car_id)
+EKFModel(const enumeration::CarIDFlag& car_id)

-// bool is_balance = ...
+bool is_balance = (car_id == enumeration::CarIDFlag::InfantryIII
+    || car_id == enumeration::CarIDFlag::InfantryIV
+    || car_id == enumeration::CarIDFlag::InfantryV);

-if (car_id == enumeration::CarIDFlag::Outpost) {
+if (is_balance) {
+    P0_dig_ << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1;
+} else if (car_id == enumeration::CarIDFlag::Outpost) {

-if (car_id == enumeration::CarIDFlag::Outpost | car_id == enumeration::CarIDFlag::Base) {
+if (is_balance) {
+    armor_num_ = 2;
+} else if (car_id == enumeration::CarIDFlag::Outpost
+    | car_id == enumeration::CarIDFlag::Base) {
     armor_num_ = 3;
```

**变更说明**:
- 移除了 `explicit` 构造函数修饰符
- 恢复了平衡步兵的特殊处理（2块装甲板）
- 平衡步兵使用独立的P0初始值

---

## 11. 状态机变更

### src/tongji/state_machine/state_machine.cpp

```diff
-void Update(std::shared_ptr<interfaces::IArmorInImage> const& armors_in_image,
-    const enumeration::CarIDFlag& invincible_armors, data::TimeStamp const& time_stamp) {
+void Update(std::shared_ptr<interfaces::IArmorInImage> armors_in_image,
+    const enumeration::CarIDFlag& invincible_armors,
+    const std::chrono::milliseconds& duration_from_last_update) {

-target_id_ =
-    tracker_->SelectTrackingTargetID(armors_in_image, invincible_armors, time_stamp);
+target_id_ = tracker_->SelectTrackingTargetID(
+    armors_in_image, invincible_armors, duration_from_last_update);
```

**变更说明**: 时间参数从绝对时间戳改为相对时间差

---

## 12. V1版本系统变更

### src/v1/auto_aim_system_v1.cpp

```diff
-core::EventBus::Subscript<data::MatStamped>
-(ParamsForSystemV1::raw_image_event, [this](const auto& mat_stamped){solve(mat_stamped);});
+core::EventBus::Subscript<cv::Mat>
+(ParamsForSystemV1::raw_image_event, [this](const auto& mat){solve(mat);});

-void solve(const data::MatStamped& mat_stamped){
-    const auto& timestamp = mat_stamped.stamp;
-    const auto& raw = mat_stamped.mat;
-    const auto& [armors, flag] = identifier->identify(raw, timestamp);
+void solve(const cv::Mat& raw){
+    const auto& [armors, flag] = identifier->identify(raw);

-identifier->SetTargetColor(ParamsForSystemV1::target_color());
+identifier->SetTargetColor(false);

-std::cerr << "[FORCE] ..." << std::endl;  // 多处调试日志被移除
```

**变更说明**:
- 事件订阅从 `MatStamped` 改为 `cv::Mat`
- `solve()` 参数从 `MatStamped` 改为 `cv::Mat`
- 目标颜色硬编码为 `false` (蓝色)
- 移除了大量调试日志

### src/v1/fire_controller/fire_controller.cpp

```diff
-: control_delay_(std::chrono::duration_cast<std::chrono::nanoseconds>(
-      std::chrono::duration<double>(control_delay_in_second)))
+: control_delay_((static_cast<time_t>(control_delay_in_second * 1e9)))

-const auto& [fly_time_ns, dir] = trajectory_solver::gravity_only(
-    candidates[static_cast<std::size_t>(index)].position, velocity_begin_, gravity_);
-data::TimeStamp fly_time { std::chrono::nanoseconds(fly_time_ns) };
-
-return { .time_stamp = data::TimeStamp { time_stamp + control_delay_ + fly_time },
-         .gimbal_dir = dir,
-         .fire_allowance = true };
+for (int i = 5; i-- > 0;) {
+    // 迭代求解飞行时间
+}
+return { .time_stamp = ..., .fire_allowance = true };
```

**变更说明**:
- 控制延迟类型转换方式改变
- 飞行时间求解添加迭代逻辑
- 返回结构移除了 `gimbal_dir` (可能是bug)

---

## 13. PnP解算器变更

### src/v1/pnpsolver/armor_pnp_solver.cpp

```diff
-armors_->time_stampe = armors->GetTimeStamp();
```

**变更说明**: 移除了时间戳设置

---

## 14. 工具类变更

### src/util/index.hpp

```diff
 if (value != 0 && (value & (value - 1)) == 0) return std::countr_zero(value);
-throw std::runtime_error("Invalid ArmorIdFlag value: " + std::to_string(value));
+else {
+    // backward stacktrace code (commented)
+    throw std::runtime_error("Invalid ArmorIdFlag value: " + std::to_string(value));
+}
```

**变更说明**: 添加了注释掉的backward堆栈追踪代码

### src/util/parameters/profile.cpp

```diff
 void set_intrinsic_matrix(...) {
-    intrinsic_parameters = (cv::Mat)(...);
-    distortion_parameters = (cv::Mat)(...);
+    intrinsic_parameters((...));
+    distortion_parameters((...));
 }
```

**变更说明**: 赋值方式改变 (可能是bug，应该是赋值而非函数调用)

---

## 总结

### 主要架构变更

1. **时间戳处理**: 从自定义 `TimeStamp` 类型大量切换到 `std::chrono` 类型，从绝对时间戳改为相对时间差
2. **接口简化**: `identify()` 接口移除了时间戳参数
3. **事件系统**: 移除了取消订阅功能
4. **路径配置**: 从相对路径改为硬编码绝对路径
5. **平衡步兵支持**: 恢复了平衡步兵的特殊处理

### 潜在问题

1. `firable_` 类型从 `bool` 改为 `double` 可能是bug
2. `profile.cpp` 中的赋值语法可能有误
3. v1 `fire_controller` 返回结构缺少 `gimbal_dir`
4. `mat_stamped.hpp` 移除了防御性类型检查可能导致断言失败
5. 硬编码路径降低了可移植性

### 新增功能

1. `CarPredictor::LastSeen()` 方法
2. 配置文件中的 `control_delay_s` 参数

### 移除功能

1. `EventBus::Unsubscribe()` 取消订阅
2. `ParamsForSystemV1::target_color()` 目标颜色参数
3. `TimeStamp` 到 `time_point` 的隐式转换
