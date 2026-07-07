#ifndef _TORQUE_DISTURBANCE_PLUGIN_HH_
#define _TORQUE_DISTURBANCE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <string>

namespace gazebo
{
class TorqueDisturbancePlugin : public ModelPlugin
{
public:
  TorqueDisturbancePlugin();
  virtual ~TorqueDisturbancePlugin();

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate();

private:
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;

  std::string link_name_;
  double amplitude_x_, amplitude_y_, amplitude_z_;
  double frequency_x_, frequency_y_, frequency_z_;

  double bias_x_, bias_y_, bias_z_;  // 新增：三轴常值偏置

  double force_amplitude_x_, force_amplitude_y_, force_amplitude_z_;
  double force_frequency_x_, force_frequency_y_, force_frequency_z_;
  double force_bias_x_, force_bias_y_, force_bias_z_;

  common::Time start_time_;
  double start_time_sec_;  // 新增：扰动开始时间（秒）
  double running_time_sec_;
};
}

#endif
