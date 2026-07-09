#include "torque_disturbance_plugin.h"
#include <ignition/math/Vector3.hh>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(TorqueDisturbancePlugin)

TorqueDisturbancePlugin::TorqueDisturbancePlugin()
{
}

TorqueDisturbancePlugin::~TorqueDisturbancePlugin()
{
}

void TorqueDisturbancePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model_ = _model;
  this->start_time_ = this->model_->GetWorld()->SimTime();

  if (_sdf->HasElement("link_name"))
    this->link_name_ = _sdf->Get<std::string>("link_name");
  else
    gzerr << "[TorqueDisturbancePlugin] Missing <link_name>\n";

  if (_sdf->HasElement("amplitude_x"))
    this->amplitude_x_ = _sdf->Get<double>("amplitude_x");

  else
    this->amplitude_x_ = 0.0;

  if (_sdf->HasElement("amplitude_y"))
    this->amplitude_y_ = _sdf->Get<double>("amplitude_y");

  else
    this->amplitude_y_ = 0.0;

  if (_sdf->HasElement("amplitude_z"))
    this->amplitude_z_ = _sdf->Get<double>("amplitude_z");

  else
    this->amplitude_z_ = 0.0;

  if (_sdf->HasElement("frequency_x"))
    this->frequency_x_ = _sdf->Get<double>("frequency_x");
  else
    this->frequency_x_ = 1.0;

  if (_sdf->HasElement("frequency_y"))
    this->frequency_y_ = _sdf->Get<double>("frequency_y");
  else
    this->frequency_y_ = 1.0;

  if (_sdf->HasElement("frequency_z"))
    this->frequency_z_ = _sdf->Get<double>("frequency_z");
  else
    this->frequency_z_ = 1.0;

  if (_sdf->HasElement("start_time"))
    this->start_time_sec_ = _sdf->Get<double>("start_time");
  else
    this->start_time_sec_ = 0.0;  // 默认立即开始扰动

  if (_sdf->HasElement("running_time"))
    this->running_time_sec_ = _sdf->Get<double>("running_time");
  else
    this->running_time_sec_ = 0.1;

  this->repeat_count_ = _sdf->Get<int>("repeat_count", 1).first;
  if (this->repeat_count_ < 1)
    this->repeat_count_ = 1;

  this->repeat_interval_sec_ = _sdf->Get<double>("repeat_interval", 0.0).first;
  if (this->repeat_interval_sec_ < 0.0)
    this->repeat_interval_sec_ = 0.0;

  this->last_repeat_index_ = -1;

  this->link_ = this->model_->GetLink(this->link_name_);
  if (!this->link_)
  {
    gzerr << "[TorqueDisturbancePlugin] Link " << this->link_name_ << " not found!\n";
    return;
  }

  this->force_amplitude_x_ = _sdf->Get<double>("force_amplitude_x", 0.0).first;
  this->force_amplitude_y_ = _sdf->Get<double>("force_amplitude_y", 0.0).first;
  this->force_amplitude_z_ = _sdf->Get<double>("force_amplitude_z", 0.0).first;

  this->force_frequency_x_ = _sdf->Get<double>("force_frequency_x", 1.0).first;
  this->force_frequency_y_ = _sdf->Get<double>("force_frequency_y", 1.0).first;
  this->force_frequency_z_ = _sdf->Get<double>("force_frequency_z", 1.0).first;

  this->force_bias_x_ = _sdf->Get<double>("force_bias_x", 0.0).first;
  this->force_bias_y_ = _sdf->Get<double>("force_bias_y", 0.0).first;
  this->force_bias_z_ = _sdf->Get<double>("force_bias_z", 0.0).first;

  this->bias_x_ = _sdf->Get<double>("bias_x", 0.0).first;
  this->bias_y_ = _sdf->Get<double>("bias_y", 0.0).first;
  this->bias_z_ = _sdf->Get<double>("bias_z", 0.0).first;

  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&TorqueDisturbancePlugin::OnUpdate, this));
}

void TorqueDisturbancePlugin::OnUpdate()
{
  common::Time current_time = this->model_->GetWorld()->SimTime();
  double t = (current_time - this->start_time_).Double();

  if (this->running_time_sec_ <= 0.0)
    return;

  if (t < this->start_time_sec_)
    return;

  double elapsed_time = t - this->start_time_sec_;
  double cycle_time = this->running_time_sec_ + this->repeat_interval_sec_;

  if (cycle_time <= 0.0)
    return;

  int repeat_index = static_cast<int>(elapsed_time / cycle_time);

  if (repeat_index >= this->repeat_count_)
    return;

  double time_in_cycle = elapsed_time - repeat_index * cycle_time;

  if (time_in_cycle > this->running_time_sec_)
    return;

  double disturbance_time = time_in_cycle;

  // 打印仿真时间（调试用）
  if (0)
  {
    gzdbg << "[TorqueDisturbancePlugin] Sim time: " << t << " s\n";
  }
  if (repeat_index != this->last_repeat_index_)
  {
    this->last_repeat_index_ = repeat_index;

    gzmsg << "[TorqueDisturbancePlugin] Disturbance repeat "
          << repeat_index + 1 << " / " << this->repeat_count_
          << ", sim time: " << t
          << ", disturbance local time: " << time_in_cycle
          << " s\n";
  }


  double tau_x = this->bias_x_ +
      this->amplitude_x_ * sin(2.0 * M_PI * this->frequency_x_ * disturbance_time);
  double tau_y = this->bias_y_ +
      this->amplitude_y_ * sin(2.0 * M_PI * this->frequency_y_ * disturbance_time);
  double tau_z = this->bias_z_ +
      this->amplitude_z_ * sin(2.0 * M_PI * this->frequency_z_ * disturbance_time);

  ignition::math::Vector3d torque(tau_x, tau_y, tau_z);
  this->link_->AddRelativeTorque(torque);

  double force_x = this->force_bias_x_ +
      this->force_amplitude_x_ * sin(2.0 * M_PI * this->force_frequency_x_ * disturbance_time);
  double force_y = this->force_bias_y_ +
      this->force_amplitude_y_ * sin(2.0 * M_PI * this->force_frequency_y_ * disturbance_time);
  double force_z = this->force_bias_z_ +
      this->force_amplitude_z_ * sin(2.0 * M_PI * this->force_frequency_z_ * disturbance_time);

  ignition::math::Vector3d force(force_x, force_y, force_z);
  this->link_->AddRelativeForce(force);

}
}
