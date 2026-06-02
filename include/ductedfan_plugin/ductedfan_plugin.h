// ======== 必要的头文件 ========
#include <stdio.h>              // 标准输入输出（如 printf）

#include <boost/bind.hpp>       // Boost 库的 bind 函数，用于创建回调函数绑定的参数占位符
#include <Eigen/Dense>         // // Eigen 线性代数库，用于控制舵面分配矩阵
#include <gazebo/gazebo.hh>     // Gazebo 通用头文件，提供核心功能
#include <gazebo/physics/physics.hh> // Gazebo 物理引擎接口（关节、刚体等）
#include <gazebo/common/common.hh>   // Gazebo 通用工具（如时间、PID、事件连接）
#include <gazebo/common/Plugin.hh>   // Gazebo 插件基类
#include <rotors_model/motor_model.hpp>  // 电机模型的基类，定义了速度滤波等通用接口
#include "CommandMotorSpeed.pb.h"     // 自定义 protobuf 消息：命令电机转速
#include "gazebo/transport/transport.hh" // Gazebo 的通信层（话题发布/订阅）
#include "gazebo/msgs/msgs.hh"        // Gazebo 预定义消息类型
#include "MotorSpeed.pb.h"            // 自定义 protobuf 消息：电机转速反馈
#include "Float.pb.h"                 // 自定义 protobuf 消息：浮点数
#include "Wind.pb.h"                  // 自定义 protobuf 消息：风速矢量

#include "common.h"                   // 包含一些通用辅助函数，如 getSdfParam
#include <vector>                     // C++ 标准库的 vector 容器
#include <string>                     // C++ 标准库的 string 类     
#include "ductedfan_plugin/spline_ppval.h" // 包含样条插值相关的定义和函数


namespace turning_direction {
// 定义旋转方向常量：CCW = 逆时针，CW = 顺时针
// 这些值与推力系数结合可产生正确的扭矩方向（用于产生偏航力矩）
const static int CCW = 1;
const static int CW = -1;
}

namespace gazebo {

// ======== 默认参数（可在 SDF 文件中覆盖） ========
static const std::string kDefaultNamespace = ""; // 默认 Gazebo 话题命名空间（空表示使用全局）
static const std::string kDefaultCommandSubTopic = "/gazebo/command/motor_speed"; // 默认命令电机转速话题
static const std::string kDefaultMotorFailureNumSubTopic = "/gazebo/motor_failure_num"; // 默认发布电机故障编号的话题
static const std::string kDefaultMotorVelocityPubTopic = "/motor_speed"; // 默认发布电机实际转速的话题
std::string wind_sub_topic_ = "/world_wind"; // 世界风话题（非 const，因为可能在运行时修改）

// 定义 protobuf 消息类型的共享指针别名，便于回调中使用
typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;
typedef const boost::shared_ptr<const physics_msgs::msgs::Wind> WindPtr;


// 以下注释是针对 Probut 测试的代码，被屏蔽
/*
// Protobuf test
typedef const boost::shared_ptr<const mav_msgs::msgs::MotorSpeed> MotorSpeedPtr;
static const std::string kDefaultMotorTestSubTopic = "motors";
*/

static constexpr double kDefaultMaxForce = std::numeric_limits<double>::max();// 默认最大力设定为最大值（实际限制由一阶滤波器处理）
static constexpr double kDefaultMotorConstant = 8.54858e-06;    // 推力系数 (N/(rad/s)^2)
static constexpr double kDefaultMomentConstant = 0.016;        // 力矩系数 (Nm/N)
static constexpr double kDefaultTimeConstantUp = 1.0 / 80.0;   // 电机加速时间常数（一阶滤波器）
static constexpr double kDefaultTimeConstantDown = 1.0 / 40.0;  // 电机减速时间常数
static constexpr double kDefaulMaxRotVelocity = 838.0;          // 最大旋转角速度 (rad/s) [约为 8000 RPM]
static constexpr double kDefaultRotorDragCoefficient = 1.0e-4; // 旋翼阻力系数（与前进比相关）
static constexpr double kDefaultRollingMomentCoefficient = 1.0e-6; // 滚转力矩系数
static constexpr double kDefaultRotorVelocitySlowdownSim = 10.0; // 仿真中降低转速的因子（为减少仿真需求而人为减慢电机）

// ======== DuctedFanModel 类 ========
// 继承自 MotorModel (基类) 和 ModelPlugin (Gazebo 模型插件接口)
class DuctedFanModel : public MotorModel, public ModelPlugin {
 public:
  // 构造函数：初始化所有成员变量为默认值
  DuctedFanModel()
      : ModelPlugin(),
        MotorModel(),
        command_sub_topic_(kDefaultCommandSubTopic),       // 默认命令电机转速话题
        motor_failure_sub_topic_(kDefaultMotorFailureNumSubTopic), // 电机故障话题
        motor_speed_pub_topic_(kDefaultMotorVelocityPubTopic),     // 转速发布话题
        motor_number_(0),           // 电机编号（从0开始）
        motor_Failure_Number_(0),   // 故障电机编号（0表示无故障）
        turning_direction_(turning_direction::CW), // 默认顺时针
        max_force_(kDefaultMaxForce),   // 默认最大力设定为最大值（实际限制由一阶滤波器处理）
        max_rot_velocity_(kDefaulMaxRotVelocity),   // 最大旋转角速度 (rad/s) [约为 8000 RPM]
        moment_constant_(kDefaultMomentConstant),   // 力矩系数 (Nm/N)
        motor_constant_(kDefaultMotorConstant),  // 推力系数 (N/(rad/s)^2)  
        // motor_test_sub_topic_(...), // 测试话题被注释
        ref_motor_rot_vel_(0.0),     // 参考角速度初始化为0
        rolling_moment_coefficient_(kDefaultRollingMomentCoefficient),  // 滚转力矩系数
        rotor_drag_coefficient_(kDefaultRotorDragCoefficient),   // 旋翼阻力系数（与前进比相关）
        rotor_velocity_slowdown_sim_(kDefaultRotorVelocitySlowdownSim), // 仿真中降低转速的因子（为减少仿真需求而人为减慢电机）
        time_constant_down_(kDefaultTimeConstantDown),  // 电机减速时间常数
        time_constant_up_(kDefaultTimeConstantUp),  // 电机加速时间常数（一阶滤波器）
        reversible_(false), // 默认不可反转（推力方向只能为正）
        control_joint_names_(kControlJointCount + 1),
        control_joints_(kControlJointCount + 1),
        control_joint_angles_(kControlJointCount + 1, 0.0),
        B_cs_(Eigen::Matrix<double, 3, 6>::Zero()){        
  }

  virtual ~DuctedFanModel(); // 虚析构函数
  
  // 插件初始化参数方法（基类接口，当前实现为空）
  virtual void InitializeParams();
  
  // 发布电机实际转速（通过 Gazebo 话题）
  virtual void Publish();

  
 protected:
  // 更新施加在刚体上的力和力矩（核心物理计算）
  virtual void UpdateForcesAndMoments();

  // 检测并模拟电机故障：如果故障编号与本电机匹配，则将电机转速强制设为0
  virtual void UpdateMotorFail();

  // 装载插件时调用：解析 SDF 参数，初始化通信和滤波器
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  // 仿真更新回调：由 Gazebo 世界更新事件触发，计算采样时间，调用上述计算和故障检查
  virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

 private:
  // ---------- 配置参数 ----------
  std::string command_sub_topic_;        // 接收期望转速的话题
  std::string motor_failure_sub_topic_;  // 接收电机故障编号的话题
  std::string joint_name_;               // 电机旋转关节的名称（SDF中指定）
  std::string link_name_;                // 旋翼对应的刚体链接名称（SDF中指定）
  std::string motor_speed_pub_topic_;    // 发布实际转速的话题
  std::string namespace_;                // 本模型的 Gazebo 命名空间（通常由机器人命名空间决定）

  int motor_number_;          // 电机编号（0~N-1）
  int turning_direction_;     // 旋转方向（CCW 或 CW）

  int motor_Failure_Number_;  // 当前应失效的电机编号（1开始，0表示无故障）
  int tmp_motor_num;          // 临时变量，用于存储上一次故障的电机编号，以便打印恢复信息

  int screen_msg_flag = 1;    // 标志位：控制故障/恢复消息只打印一次，避免刷屏

  // ---------- 物理参数 ----------
  double max_force_;                 // 非PID模式下的最大力（在 Gazebo 5 以前使用，现已不使用）
  double max_rot_velocity_;          // 最大参考角速度 (rad/s)
  double moment_constant_;           // 扭矩-推力系数 (Nm/N)
  double motor_constant_;            // 推力-转速系数 (N/(rad/s)^2)
  double ref_motor_rot_vel_;         // 期望电机角速度（从命令话题接收）
  double rolling_moment_coefficient_; // 滚转力矩系数
  double rotor_drag_coefficient_;     // 旋翼叶片阻力系数
  double rotor_velocity_slowdown_sim_; // 实际物理转速 = 仿真关节速度 * 此系数，用于模拟真实高速旋转
  double time_constant_down_;         // 减速一阶滤波时间常数
  double time_constant_up_;           // 加速一阶滤波时间常数

  bool reversible_;                   // 是否允许反向推力（例如用于倒飞特技）

  bool fly;                           // 一个未使用的标志，原代码中有遗留

  // MATLAB 涵道模型新增参数
  double k_Th_;        // thrustVelocityCoupling
  double k_Ts_;        // thrustAoaCoupling
  double k_Ns_;        // sideForceVelocityCoupling
  double l_cpz_;       // sideForceArmZ
  double k_cpx_;       // thrustCenterOffsetCoeff

  double duct_sd_;   // 涵道扩压比
  double duct_S_;    // 桨盘面积
  double air_density_; // 空气密度

  double k_my0_;   // 俯仰阻尼常数项
  double k_myv_;   // 俯仰阻尼速度项系数

  double k_sta_;   // 涵道反扭矩系数
  double I_fan_;   // 风扇转动惯量

  std::vector<SplineSegment> spline_dt_;    // 涵道拉力系数样条
  std::vector<SplineSegment> spline_dn_;    // 涵道侧向力系数样条
  
  std::vector<SplineSegment> wing_spline_wl_; // 机翼升力系数样条
  std::vector<SplineSegment> wing_spline_wd_; // 机翼侧向力系数样条
  std::vector<SplineSegment> wing_spline_wm_; // 机翼俯仰力矩系数样条


  //控制舵面角度读取区
  static const int kControlJointCount = 6;  // 控制舵面数量，编号为 1 到 6

  std::vector<std::string> control_joint_names_; // 控制舵面关节名称，使用下标 1 到 6

  std::vector<physics::JointPtr> control_joints_; // 控制舵面关节指针

  std::vector<double> control_joint_angles_;  // 控制舵面当前转角，单位 rad

  void LoadControlJoints(sdf::ElementPtr _sdf);  // 读取 SDF 中的控制舵面关节

  void ReadControlJointAngles();                 // 读取当前控制舵面角度

  void LoadControlEffectivenessMatrix(sdf::ElementPtr _sdf);  // 从 SDF 读取控制舵面分配矩阵

  double d_cs_; // 单个控制舵面的力系数

  double l1_cs_;  // 控制舵面力臂参数 l1

  double l2_cs_;  // 控制舵面力臂参数 l2

  Eigen::Matrix<double, 3, 6> B_cs_;            // 控制舵面分配矩阵，维度为 3 x 6

  // ---------- 通信与 Gazebo 对象 ----------
  transport::NodePtr node_handle_;        // Gazebo 通信节点
  transport::PublisherPtr motor_velocity_pub_; // 发布电机实际转速的发布者
  transport::SubscriberPtr command_sub_;      // 订阅期望转速
  transport::SubscriberPtr motor_failure_sub_; // 订阅电机故障编号
  transport::SubscriberPtr wind_sub_;          // 订阅世界风速

  ignition::math::Vector3d wind_vel_;   // 存储当前风速矢量

  physics::ModelPtr model_;        // 当前模型（无人机整体）指针
  physics::JointPtr joint_;        // 电机旋转关节的指针
  common::PID pid_;                // 速度闭环 PID 控制器（如果启用）
  bool use_pid_;                   // 是否使用 PID 控制关节力来跟踪转速
  physics::LinkPtr link_;          // 旋翼刚体的指针

  physics::LinkPtr base_link_;   // 机体基准连杆，用于读取运动状态和施加合力
  
  /// Gazebo 事件连接（用于更新回调）
  event::ConnectionPtr updateConnection_;

  // 回调线程相关（原代码未实际使用）
  boost::thread callback_queue_thread_;
  void QueueThread();

  // 存储要发布的转速消息（protobuf float类型）
  std_msgs::msgs::Float turning_velocity_msg_;

  // ======== 回调函数 ========
  // 命令转速回调：解析消息，将对应电机的期望转速存入 ref_motor_rot_vel_，并限制在最大值内
  void VelocityCallback(CommandMotorSpeedPtr &rot_velocities);

  // 电机故障回调：接收整数消息（故障电机编号），存入 motor_Failure_Number_
  void MotorFailureCallback(const boost::shared_ptr<const msgs::Int> &fail_msg);

  // 风速回调：接收 Wind 消息，更新世界风速矢量 wind_vel_
  void WindVelocityCallback(const boost::shared_ptr<const physics_msgs::msgs::Wind> &msg);

  // 一阶低通滤波器对象，用于模拟电机响应延迟
  std::unique_ptr<FirstOrderFilter<double>>  rotor_velocity_filter_;

  // 以下测试订阅被注释
  /*
  std::string motor_test_sub_topic_;
  transport::SubscriberPtr motor_sub_;
  */
};
} // namespace gazebo