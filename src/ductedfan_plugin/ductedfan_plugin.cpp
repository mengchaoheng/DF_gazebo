
#include "ductedfan_plugin/ductedfan_plugin.h" //ductedfan_plugin/ductedfan_plugin.h实际上用的这个
#include <ignition/math.hh>  // Ignition Math 数学库，提供 Vector3、Pose3 等
#include <cmath>
#include <sstream>   // 用于解析 SDF 中的矩阵字符串
#include "ductedfan_plugin/spline_ppval.h" // 包含样条插值相关的定义和PpvalSpline函数

// 以下两个为测试时打印变量使用的头文件，以后可以直接删去==
#include <iostream> // 确保包含了 iostream 头文件
#include <chrono> // 如果还没有，请加上这一行
//====================================================

namespace gazebo {
namespace {
ignition::math::Vector3d FluToFrd(const ignition::math::Vector3d& v)
{
  return {v.X(), -v.Y(), -v.Z()};
}

ignition::math::Vector3d FrdToFlu(const ignition::math::Vector3d& v)
{
  return {v.X(), -v.Y(), -v.Z()};
}
} // namespace

// 析构函数：断开更新事件连接，并关闭 PID 模式
DuctedFanModel::~DuctedFanModel() {
  updateConnection_->~Connection(); // 显式断开连接（实际可以简单 reset）
  use_pid_ = false;
}

// 初始化参数虚函数，这里空实现
void DuctedFanModel::InitializeParams() {}

// 发布电机实际转速（TODO: 当前发布功能被注释掉，防止队列警告）
void DuctedFanModel::Publish() {
  // 获取关节当前速度（rad/s）
  turning_velocity_msg_.set_data(joint_->GetVelocity(0));
  // FIXME: 为防止警告“queue limit reached”，发布被注释。
  // motor_velocity_pub_->Publish(turning_velocity_msg_); //turning_velocity_msg_ 存储要发布的转速消息
}

// 装载插件：解析 SDF 参数，获取仿真对象，初始化通信和滤波器
void DuctedFanModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;

  namespace_.clear();
  // 读取机器人命名空间
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[ductedfan_plugin model] Please specify a robotNamespace.\n";

  // 初始化 Gazebo 通信节点
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  // ---------- 读取 SDF 中配置的关节和链接名称 ----------
  if (_sdf->HasElement("jointName"))
    joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
  else
    gzerr << "[ductedfan_plugin model] Please specify a jointName, where the rotor is attached.\n";
  joint_ = model_->GetJoint(joint_name_);
  if (joint_ == NULL)
    gzthrow("[ductedfan_plugin model] Couldn't find specified joint \"" << joint_name_ << "\".");

  // 配置关节 PID 控制器（可选，用于力控制模式下跟踪期望转速）
  if (_sdf->HasElement("joint_control_pid")) {
    sdf::ElementPtr pid = _sdf->GetElement("joint_control_pid");
    double p = 0.1;
    if (pid->HasElement("p")) p = pid->Get<double>("p");
    double i = 0;
    if (pid->HasElement("i")) i = pid->Get<double>("i");
    double d = 0;
    if (pid->HasElement("d")) d = pid->Get<double>("d");
    double iMax = 0;
    if (pid->HasElement("iMax")) iMax = pid->Get<double>("iMax");
    double iMin = 0;
    if (pid->HasElement("iMin")) iMin = pid->Get<double>("iMin");
    double cmdMax = 3;
    if (pid->HasElement("cmdMax")) cmdMax = pid->Get<double>("cmdMax");
    double cmdMin = -3;
    if (pid->HasElement("cmdMin")) cmdMin = pid->Get<double>("cmdMin");
    pid_.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
    use_pid_ = true;
  } else {
    use_pid_ = false;  // 默认不使用 PID，直接设定关节速度
  }

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[ductedfan_plugin model] Please specify a linkName of the rotor.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[ductedfan_plugin model] Couldn't find specified link \"" << link_name_ << "\".");

  // 获取机体基准连杆（通常为 base_link），用于后续施加力和读取状态
  base_link_ = model_->GetLink("base_link");
  if (!base_link_) {
    gzerr << "[ductedfan_plugin] 无法找到 base_link，机体气动力和力矩将无法施加！\n";
  }

  // 电机编号
  if (_sdf->HasElement("motorNumber"))
    motor_number_ = _sdf->GetElement("motorNumber")->Get<int>();
  else
    gzerr << "[ductedfan_plugin model] Please specify a motorNumber.\n";

  // 旋翼旋转方向
  if (_sdf->HasElement("turningDirection")) {
    std::string turning_direction = _sdf->GetElement("turningDirection")->Get<std::string>();
    if (turning_direction == "cw")
      turning_direction_ = turning_direction::CW;
    else if (turning_direction == "ccw")
      turning_direction_ = turning_direction::CCW;
    else
      gzerr << "[ductedfan_plugin model] Please only use 'cw' or 'ccw' as turningDirection.\n";
  } else {
    gzerr << "[ductedfan_plugin model] Please specify a turning direction ('cw' or 'ccw').\n";
  }

  // 是否允许旋翼反转
  if(_sdf->HasElement("reversible")) {
    reversible_ = _sdf->GetElement("reversible")->Get<bool>();
  }

  // ---------- 加载涵道气动样条系数（dt, dn） ----------
  if (_sdf->HasElement("ductSplineDtFile")) {
      std::string dt_path = _sdf->GetElement("ductSplineDtFile")->Get<std::string>();
      if (!LoadSplineFromCsv(dt_path, spline_dt_)) {
          gzerr << "[ductedfan_plugin] 加载涵道拉力系数样条失败，文件: " << dt_path << "\n";
      }
  } else {
      gzerr << "[ductedfan_plugin] 未指定 ductSplineDtFile，将不使用样条修正推力\n";
  }

  if (_sdf->HasElement("ductSplineDnFile")) {
      std::string dn_path = _sdf->GetElement("ductSplineDnFile")->Get<std::string>();
      if (!LoadSplineFromCsv(dn_path, spline_dn_)) {
          gzerr << "[ductedfan_plugin] 加载涵道侧向力系数样条失败，文件: " << dn_path << "\n";
      }
  } else {
      gzerr << "[ductedfan_plugin] 未指定 ductSplineDnFile，侧向力将不采用样条\n";
  }

  // ---------- 加载机翼气动样条系数（wl, wd, wm） ----------
  if (_sdf->HasElement("wingSplineWlFile")) {
      std::string wl_path = _sdf->GetElement("wingSplineWlFile")->Get<std::string>();
      if (!LoadSplineFromCsv(wl_path, wing_spline_wl_))
          gzerr << "[ductedfan_plugin] 加载机翼升力样条失败！\n";
  } else {
      gzerr << "[ductedfan_plugin] 未指定 wingSplineWlFile\n";
  }
  // 同样加载 wd 和 wm
  if (_sdf->HasElement("wingSplineWdFile")) {
      std::string wd_path = _sdf->GetElement("wingSplineWdFile")->Get<std::string>();
      if (!LoadSplineFromCsv(wd_path, wing_spline_wd_))
          gzerr << "[ductedfan_plugin] 加载机翼阻力样条失败！\n";
  } else {
      gzerr << "[ductedfan_plugin] 未指定 wingSplineWdFile\n";
  }
  if (_sdf->HasElement("wingSplineWmFile")) {
      std::string wm_path = _sdf->GetElement("wingSplineWmFile")->Get<std::string>();
      if (!LoadSplineFromCsv(wm_path, wing_spline_wm_))
          gzerr << "[ductedfan_plugin] 加载机翼俯仰力矩样条失败！\n";
  } else {
      gzerr << "[ductedfan_plugin] 未指定 wingSplineWmFile\n";
  }

  // 读取 1 到 6 号控制舵面关节
  LoadControlJoints(_sdf);

  // 读取控制舵面分配矩阵 B_cs_
  LoadControlEffectivenessMatrix(_sdf);

  // ---------- 使用 common.h 中的辅助函数读取可选参数 ----------
  getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);
  getSdfParam<std::string>(_sdf, "motorSpeedPubTopic", motor_speed_pub_topic_, motor_speed_pub_topic_);

  getSdfParam<double>(_sdf, "rotorDragCoefficient", rotor_drag_coefficient_, rotor_drag_coefficient_);
  getSdfParam<double>(_sdf, "rollingMomentCoefficient", rolling_moment_coefficient_, rolling_moment_coefficient_);
  getSdfParam<double>(_sdf, "maxRotVelocity", max_rot_velocity_, max_rot_velocity_);
  getSdfParam<double>(_sdf, "motorConstant", motor_constant_, motor_constant_);
  getSdfParam<double>(_sdf, "momentConstant", moment_constant_, moment_constant_);

  getSdfParam<double>(_sdf, "timeConstantUp", time_constant_up_, time_constant_up_);
  getSdfParam<double>(_sdf, "timeConstantDown", time_constant_down_, time_constant_down_);
  getSdfParam<double>(_sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_, 10);

  getSdfParam<double>(_sdf, "thrustVelocityCoupling", k_Th_, 0.0);
  getSdfParam<double>(_sdf, "thrustAoaCoupling", k_Ts_, 0.0);
  getSdfParam<double>(_sdf, "sideForceVelocityCoupling", k_Ns_, 0.0);
  getSdfParam<double>(_sdf, "sideForceArmZ", l_cpz_, 0.0);
  getSdfParam<double>(_sdf, "thrustCenterOffsetCoeff", k_cpx_, 0.0);

  getSdfParam<double>(_sdf, "ductExpansionRatio", duct_sd_, 0.7);
  getSdfParam<double>(_sdf, "ductDiskArea", duct_S_, M_PI * 0.114 * 0.114);
  getSdfParam<double>(_sdf, "airDensity", air_density_, 1.225);

  getSdfParam<double>(_sdf, "pitchDampingConstant", k_my0_, 0.0);
  getSdfParam<double>(_sdf, "pitchDampingVelocityCoeff", k_myv_, 0.0);

  getSdfParam<double>(_sdf, "ductTorqueCoeff", k_sta_, 0.0);
  getSdfParam<double>(_sdf, "fanInertia", I_fan_, 0.0);

  // 读取控制舵面力矩模型参数
  getSdfParam<double>(_sdf, "control_surface_force_coeff", d_cs_, 0.0);
  getSdfParam<double>(_sdf, "control_surface_arm_l1", l1_cs_, 0.0);
  getSdfParam<double>(_sdf, "control_surface_arm_l2", l2_cs_, 0.0);

  fly = false;  // 某个未使用的标志

  // 在 Gazebo 5 以前的版本，设置关节最大力（以后版本不再使用此接口）
#if GAZEBO_MAJOR_VERSION < 5
  joint_->SetMaxForce(0, max_force_);
#endif

  // 连接世界更新事件：每个仿真步调用 OnUpdate
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&DuctedFanModel::OnUpdate, this, _1));

  // 订阅命令电机转速话题（话题格式：~/模型名/command_sub_topic_）
  command_sub_ = node_handle_->Subscribe<mav_msgs::msgs::CommandMotorSpeed>(
      "~/" + model_->GetName() + command_sub_topic_, &DuctedFanModel::VelocityCallback, this);

  // 订阅电机故障话题
  motor_failure_sub_ = node_handle_->Subscribe<msgs::Int>(
      motor_failure_sub_topic_, &DuctedFanModel::MotorFailureCallback, this);

  // 发布实际电机转速话题（当前发布被注释，但发布者对象仍可创建）
  // motor_velocity_pub_ = node_handle_->Advertise<std_msgs::msgs::Float>("~/" + model_->GetName() + motor_speed_pub_topic_, 1);

  // 订阅世界风话题
  wind_sub_ = node_handle_->Subscribe("~/" + wind_sub_topic_, &DuctedFanModel::WindVelocityCallback, this);

  // 初始化一阶滤波器：使用加速/减速时间常数，初始参考转速为0
  rotor_velocity_filter_.reset(new FirstOrderFilter<double>(time_constant_up_, time_constant_down_, ref_motor_rot_vel_));
}



// 读取控制舵面关节名称并获取关节指针
void DuctedFanModel::LoadControlJoints(sdf::ElementPtr _sdf)
{
  for (int i = 1; i <= kControlJointCount; ++i) {
    std::string sdf_tag = "control_joint_name_" + std::to_string(i);

    if (!_sdf->HasElement(sdf_tag)) {
      gzerr << "[ductedfan_plugin] 未设置 " << sdf_tag
            << "，CS" << i << " 角度默认为 0。\n";
      continue;
    }

    std::string joint_name = _sdf->Get<std::string>(sdf_tag);
    physics::JointPtr joint = model_->GetJoint(joint_name);

    if (!joint) {
      gzerr << "[ductedfan_plugin] 无法找到控制舵面关节: "
            << joint_name << "\n";
      continue;
    }

    control_joint_names_[i] = joint_name;
    control_joints_[i] = joint;
    control_joint_angles_[i] = 0.0;

    gzmsg << "[ductedfan_plugin] 已加载控制舵面关节 CS"
          << i << ": " << joint_name << "\n";
  }
}
// 读取 1 到 6 号控制舵面当前角度
void DuctedFanModel::ReadControlJointAngles()
{
  for (int i = 1; i <= kControlJointCount; ++i) {
    if (!control_joints_[i]) {
      control_joint_angles_[i] = 0.0;
      continue;
    }

#if GAZEBO_MAJOR_VERSION >= 9
    control_joint_angles_[i] = control_joints_[i]->Position(0);
#else
    control_joint_angles_[i] = control_joints_[i]->GetAngle(0).Radian();
#endif
  }
}
// 从 SDF 读取控制舵面分配矩阵 B_cs_，维度为 3 x 6
void DuctedFanModel::LoadControlEffectivenessMatrix(sdf::ElementPtr _sdf)
{
  if (!_sdf->HasElement("control_effectiveness_matrix")) {
    gzerr << "[ductedfan_plugin] 未设置 control_effectiveness_matrix，B_cs_ 使用零矩阵。\n";
    B_cs_.setZero();
    return;
  }

  std::string matrix_str = _sdf->GetElement("control_effectiveness_matrix")->Get<std::string>();
  std::stringstream ss(matrix_str);

  std::vector<double> values;
  double value = 0.0;

  while (ss >> value) {
    values.push_back(value);
  }

  if (values.size() != 18) {
    gzerr << "[ductedfan_plugin] control_effectiveness_matrix 参数数量错误，应为 18 个，实际为 "
          << values.size() << " 个。B_cs_ 使用零矩阵。\n";
    B_cs_.setZero();
    return;
  }

  int index = 0;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 6; ++col) {
      B_cs_(row, col) = values[index];
      ++index;
    }
  }

  gzmsg << "[ductedfan_plugin] 控制舵面分配矩阵 B_cs_ 读取完成：\n"
        << B_cs_ << "\n";
}

// ======== 以下为各种回调 ========

// 世界更新开始回调
void DuctedFanModel::OnUpdate(const common::UpdateInfo& _info) {
  // 计算采样时间（当前仿真时间 - 上一仿真时间）
  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();

  // 执行力和力矩计算（物理更新）
  UpdateForcesAndMoments();
  // 检查电机故障并处理
  UpdateMotorFail();
  // 发布实际转速
  Publish();
}

// 接收电机转速命令
void DuctedFanModel::VelocityCallback(CommandMotorSpeedPtr &rot_velocities) {
  // 检查数组大小是否足够
  if(rot_velocities->motor_speed_size() < motor_number_) {
    std::cout << "You tried to access index " << motor_number_
              << " of the MotorSpeed message array which is of size "
              << rot_velocities->motor_speed_size() << "." << std::endl;
  } else {
    // 将期望转速限制在最大转速内，然后赋值给 ref_motor_rot_vel_
    ref_motor_rot_vel_ = std::min(
        static_cast<double>(rot_velocities->motor_speed(motor_number_)),
        static_cast<double>(max_rot_velocity_));
  }
}

// 接收电机故障消息
void DuctedFanModel::MotorFailureCallback(const boost::shared_ptr<const msgs::Int> &fail_msg) {
  motor_Failure_Number_ = fail_msg->data();
}

// 接收风速消息
void DuctedFanModel::WindVelocityCallback(WindPtr& msg) {
  wind_vel_ = ignition::math::Vector3d(
      msg->velocity().x(),
      msg->velocity().y(),
      msg->velocity().z());
}

// ======== 核心：更新作用在旋翼上的力和力矩 ========
void DuctedFanModel::UpdateForcesAndMoments() {
  // 获取当前关节角速度（rad/s）
  motor_rot_vel_ = joint_->GetVelocity(0);

  // 读取 1 到 6 号控制舵面当前角度，单位 rad，舵序号保持 SDF/PX4 顺序不重排。
  ReadControlJointAngles();

  // 将仿真关节速度映射回物理真实转速。仿真中为了避免过高频率而降低了转子显示转速。
  const double real_motor_velocity = motor_rot_vel_ * rotor_velocity_slowdown_sim_;
  const double omega = std::abs(real_motor_velocity);

  // 检查混叠风险：如果仿真步长太大导致转速过高，可能无法准确捕捉旋转。
  if (sampling_time_ > 0.0 && omega / (2.0 * M_PI) > 1.0 / (2.0 * sampling_time_)) {
    gzerr << "Aliasing on motor [" << motor_number_
          << "] might occur. Consider making smaller simulation time steps or raising the rotor_velocity_slowdown_sim_ param.\n";
  }

  // 获取机体基准连杆在世界坐标系下的线速度、姿态和角速度。
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d body_velocity = base_link_->WorldLinearVel();
  ignition::math::Pose3d link_pose = base_link_->WorldPose();
  ignition::math::Vector3d ang_vel_world = base_link_->WorldAngularVel();
#else
  ignition::math::Vector3d body_velocity = ignitionFromGazeboMath(base_link_->GetWorldLinearVel());
  ignition::math::Pose3d link_pose = ignitionFromGazeboMath(base_link_->GetWorldPose());
  ignition::math::Vector3d ang_vel_world = ignitionFromGazeboMath(base_link_->GetWorldAngularVel());
#endif

  // Gazebo body 坐标为 FLU（x前、y左、z上），建模统一转成 FRD（x前、y右、z下）。
  const auto airspeed_frd = FluToFrd(link_pose.Rot().RotateVectorReverse(body_velocity - wind_vel_));
  const auto rates_frd = FluToFrd(link_pose.Rot().RotateVectorReverse(ang_vel_world));
  const double u = airspeed_frd.X();
  const double v = airspeed_frd.Y();
  const double w = airspeed_frd.Z();
  const double p = rates_frd.X();
  const double q = rates_frd.Y();
  const double r = rates_frd.Z();

  // 构造 FRD 机体系下的控制舵面角度向量，单位 rad。
  Eigen::Matrix<double, 6, 1> control;
  for (int i = 0; i < kControlJointCount; ++i) {
    control(i) = control_joint_angles_[i + 1];
  }

  //--------------------------------------------------------
  // 空速、迎角（FRD 机体系）
  const double Vxz = std::sqrt(u * u + w * w);
  const double Vxy = std::sqrt(u * u + v * v);
  const double V = std::sqrt(u * u + v * v + w * w);
  const double inv_Vxy = Vxy > 1e-12 ? 1.0 / Vxy : 0.0;
  const double x_dir = Vxy > 1e-12 ? u * inv_Vxy : 1.0;
  const double y_dir = Vxy > 1e-12 ? v * inv_Vxy : 0.0;

  double AOA = 0.0;
  if (Vxz <= 1.0) {
    AOA = M_PI / 2.0;
  } else {
    double cosA = -w / Vxz;
    cosA = ignition::math::clamp(cosA, -1.0, 1.0);
    AOA = std::acos(cosA);
  }

  const double duct_cos_aoa = V > 1e-6 ? ignition::math::clamp(-w / V, -1.0, 1.0) : 0.0;
  const double duct_sin_aoa = V > 1e-6 ? Vxy / V : 1.0;
  const double duct_aoa = V > 1e-6 ? std::acos(duct_cos_aoa) : M_PI / 2.0;

  // ================= Ducted_Fan_FnM 函数 =================
  // 输入：转速 omega，来流速度分量 u, v, w（FRD）
  // 输出：推力、侧向力、涵道俯仰力矩、反扭矩。
  double dt = 0.0, dn = 0.0;
  if (!spline_dt_.empty()) dt = PpvalSpline(spline_dt_, duct_aoa, true);
  if (!spline_dn_.empty()) dn = PpvalSpline(spline_dn_, duct_aoa, true);

  const double omega2 = omega * omega;
  double duct_thrust = motor_constant_ * omega2
      + V * omega * (k_Th_ + k_Ts_ * duct_cos_aoa)
      + V * V * dt;

  if (!reversible_) {
    duct_thrust = std::abs(duct_thrust);
  }

  const double duct_side_force = V * omega * k_Ns_ * duct_sin_aoa + V * V * dn;
  double duct_pitch_moment = duct_side_force * l_cpz_;
  if (omega > 1e-6) {
    duct_pitch_moment += duct_thrust * k_cpx_ * V / omega * duct_sin_aoa;
  }

  // ================= Wing_FnM 函数 =================
  // 机翼样条输出已经投影到 FRD 的 x-z 平面：x 为前向、z 为下向，绕 y 为俯仰力矩。
  double wl = 0.0, wd = 0.0, wm = 0.0;
  if (!wing_spline_wl_.empty()) wl = PpvalSpline(wing_spline_wl_, AOA, true);
  if (!wing_spline_wd_.empty()) wd = PpvalSpline(wing_spline_wd_, AOA, true);
  if (!wing_spline_wm_.empty()) wm = PpvalSpline(wing_spline_wm_, AOA, true);

  const double Vxz2 = Vxz * Vxz;
  const double wing_force_x = Vxz2 * wl;
  const double wing_force_z = Vxz2 * wd;
  const double wing_pitch_moment = Vxz2 * wm;
  const double Ve = -0.5 * w + std::sqrt(0.25 * w * w + duct_thrust / (duct_sd_ * air_density_ * duct_S_));
  const double rotor_torque = moment_constant_ * omega2;

  // ================= 计算合力和合力矩（FRD） =================
  const ignition::math::Vector3d force_frd(
      -duct_side_force * x_dir - wing_force_x,
      -duct_side_force * y_dir,
      -duct_thrust - wing_force_z);

  Eigen::Matrix3d L_cs = Eigen::Matrix3d::Zero();
  L_cs(0, 0) = l1_cs_;
  L_cs(1, 1) = l1_cs_;
  L_cs(2, 2) = l2_cs_;

  const Eigen::Matrix<double, 3, 1> M_cs = d_cs_ * Ve * Ve * L_cs * B_cs_ * control;
  const ignition::math::Vector3d torque_frd(
      -duct_pitch_moment * y_dir - turning_direction_ * I_fan_ * omega * q + M_cs(0),
       duct_pitch_moment * x_dir + wing_pitch_moment - q * (k_my0_ + k_myv_ * Vxz)
           + turning_direction_ * I_fan_ * omega * p + M_cs(1),
      -turning_direction_ * rotor_torque + turning_direction_ * k_sta_ * Ve * Ve + M_cs(2));

  // 转换到 Gazebo body 坐标系 FLU，只在输出给 Gazebo 时做一次坐标变换。
  const auto force_flu = FrdToFlu(force_frd);
  const auto torque_flu = FrdToFlu(torque_frd);

  // ================= 施加力与力矩到 base_link =================
  if (base_link_) {
    base_link_->AddRelativeForce(force_flu);
    base_link_->AddRelativeTorque(torque_flu);
  }

  // ================= 速度控制（设定关节速度） =================
  const double ref_motor_rot_vel = rotor_velocity_filter_->updateFilter(ref_motor_rot_vel_, sampling_time_);

#if 0 //FIXME: disable PID for now, it does not play nice with the PX4 CI system.
  if (use_pid_) {
    double err = joint_->GetVelocity(0) - turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_;
    double rotorForce = pid_.Update(err, sampling_time_);
    joint_->SetForce(0, rotorForce);
  } else {
    // Gazebo 7+ 直接设置速度
    joint_->SetVelocity(0, turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_);
  }
#else
  joint_->SetVelocity(0, turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_);
#endif

  // ================= 调试打印部分 =========================
  // 使用 static 保证变量在多次调用中保持值，用于记录上次打印时间。
  // DEBUG_PRINT 可以手动控制是否启用调试打印。
  const bool DEBUG_PRINT = false; // 手动控制
  if (DEBUG_PRINT){
    static std::chrono::steady_clock::time_point last_print_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();

    // 计算流逝时间（毫秒）
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_print_time).count();

    // 设定间隔阈值：1000 毫秒 (1秒)
    if (elapsed_ms >= 1000) {
        std::cout << "--- Debug Info [Motor " << motor_number_ << "] ---" << std::endl;
        std::cout << "real_motor_velocity: " << real_motor_velocity << "\n"
          << "AOA(deg): " << AOA * 180.0 / M_PI
          << " wl:" << wl << " wd:" << wd << " wm:" << wm << "\n"
          << "duct_AOA(deg): " << duct_aoa * 180.0 / M_PI
          << " dt:" << dt << " dn:" << dn << "\n"
          << "duct_thrust: " << duct_thrust << " duct_side_force: " << duct_side_force
          << " duct_pitch_moment: " << duct_pitch_moment << " rotor_torque: " << rotor_torque << "\n"
          << "Ve: " << Ve << " m/s\n"
          << "p_frd:" << p << " q_frd:" << q << " r_frd:" << r << "\n"
          << "M_cs: "<< M_cs(0) << " "<< M_cs(1) << " "<< M_cs(2) << "\n"
          << "CS_frd(rad): "
          << control(0) << " " << control(1) << " " << control(2) << " "
          << control(3) << " " << control(4) << " " << control(5) << "\n"
          << "force_frd: " << force_frd << "\n"
          << "torque_frd: " << torque_frd << "\n"
          << "force_flu: " << force_flu << "\n"
          << "torque_flu: " << torque_flu << std::endl;
        std::cout << "----------------------------------" << std::endl;

        // 更新时间戳
        last_print_time = current_time;
    }
  }
  // =======================================================
}

// ======== 电机故障模拟 ========
void DuctedFanModel::UpdateMotorFail() {
  // 如果故障编号等于本电机编号+1（故障编号从1开始），则将该电机转速强制设为0
  if (motor_number_ == motor_Failure_Number_ - 1) {
    // 另一种可能的方法是将电机常数设为0，但这里直接锁定关节速度
    joint_->SetVelocity(0, 0);
    if (screen_msg_flag) {
      // 以防消息刷屏，只打印一次故障信息
      std::cout << "Motor number [" << motor_Failure_Number_ << "] failed!  [Motor thrust = 0]" << std::endl;
      tmp_motor_num = motor_Failure_Number_;
      screen_msg_flag = 0; // 已打印过故障信息
    }
  } else if (motor_Failure_Number_ == 0 && motor_number_ == tmp_motor_num - 1) {
    // 当故障编号重新置零且本电机就是之前故障的电机，打印恢复信息
    if (!screen_msg_flag) {
      std::cout << "Motor number [" << tmp_motor_num << "] running! [Motor thrust = (default)]" << std::endl;
      screen_msg_flag = 1; // 恢复标志，允许下次再打印
    }
  }
}

// 注册插件：让 Gazebo 知道这个模型插件可以加载
GZ_REGISTER_MODEL_PLUGIN(DuctedFanModel);
} // namespace gazebo
