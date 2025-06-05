#include "torque_disturbance_plugin.hh"
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

  this->link_ = this->model_->GetLink(this->link_name_);
  if (!this->link_)
  {
    gzerr << "[TorqueDisturbancePlugin] Link " << this->link_name_ << " not found!\n";
    return;
  }

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

  if (t < this->start_time_sec_)
    return;

  // 打印仿真时间（调试用）
  if (0)
  {
    gzdbg << "[TorqueDisturbancePlugin] Sim time: " << t << " s\n";
  }
  double tau_x = this->bias_x_ + this->amplitude_x_ * sin(2.0 * M_PI * this->frequency_x_ * (t - this->start_time_sec_));
  double tau_y = this->bias_y_ + this->amplitude_y_ * sin(2.0 * M_PI * this->frequency_y_ * (t - this->start_time_sec_));
  double tau_z = this->bias_z_ + this->amplitude_z_ * sin(2.0 * M_PI * this->frequency_z_ * (t - this->start_time_sec_));

  ignition::math::Vector3d torque(tau_x, tau_y, tau_z);
  this->link_->AddRelativeTorque(torque);
}
}
