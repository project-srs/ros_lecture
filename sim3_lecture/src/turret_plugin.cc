#include "turret_plugin.hh"
#include <gazebo/common/Events.hh>
#include <cmath>

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(TurretPlugin)

TurretPlugin::TurretPlugin() : nh_() {}
TurretPlugin::~TurretPlugin(){}

void TurretPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
    gzmsg << "Load TurretPlugin\n";
    GetParam(_sdf);
    model_ = _model;
    world_ = _model->GetWorld();
    link_ = _model->GetLink("link");
    update_conn_ = event::Events::ConnectWorldUpdateBegin(std::bind(&TurretPlugin::OnUpdate, this));
    point_sub_ = nh_.subscribe(topic_name_, 10, &TurretPlugin::PointCallback, this);
}

void TurretPlugin::OnUpdate() {
  common::Time current_time = world_->GetSimTime();
  if((current_time - last_time_).Double() > (1.0/2.0)){
      last_time_ = current_time;
      ignition::math::Pose3d pose = this->link_->GetWorldPose().Ign();
      printf("pos: %f %f %f\n", pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
      printf("rot: %f %f %f\n", pose.Rot().Roll(), pose.Rot().Pitch(), pose.Rot().Yaw());
  }  

  auto yaw = model_->GetJoint(yaw_joint_name_);
  yaw->SetVelocity(0, -yaw_p_ * (yaw->GetAngle(0).Radian() - yaw_target_));  
  auto pitch = model_->GetJoint(pitch_joint_name_);
  pitch->SetVelocity(0, -pitch_p_ * (pitch->GetAngle(0).Radian() - pitch_target_));  
}

void TurretPlugin::Reset() {
}

void TurretPlugin::PointCallback(const geometry_msgs::Point& point_msg){
  float hlen = sqrt(point_msg.x * point_msg.x + point_msg.y * point_msg.y);
  if(0.1 <= hlen){
    yaw_target_ = atan2(point_msg.y, point_msg.x);
    pitch_target_ = -atan2(point_msg.z, hlen);
  } 
}

bool TurretPlugin::GetParam(sdf::ElementPtr sdf){
if (!sdf->HasElement("topic_name")){
    gzmsg << "topic_name not set\n";
    return false;
  }
  else{
    topic_name_ = sdf->GetElement("topic_name")->Get<std::string>();
  }

  if (!sdf->HasElement("yaw_joint_name")){
    gzmsg << "yaw_joint_name not set\n";
    return false;
  }
  else{
    yaw_joint_name_ = sdf->GetElement("yaw_joint_name")->Get<std::string>();
  } 
  if (!sdf->HasElement("yaw_p")){
    gzmsg << "yaw_p not set\n";
    return false;
  }
  else yaw_p_ = sdf->GetElement("yaw_p")->Get<float>();

  if (!sdf->HasElement("pitch_joint_name")){
    gzmsg << "pitch_joint_name not set\n";
    return false;
  }
  else{
    pitch_joint_name_ = sdf->GetElement("pitch_joint_name")->Get<std::string>();
  } 
  if (!sdf->HasElement("pitch_p")){
    gzmsg << "pitch_p not set\n";
    return false;
  }
  else pitch_p_ = sdf->GetElement("pitch_p")->Get<float>();

  return true;
}
} // end namespace gazebo