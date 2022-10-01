#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
class GAZEBO_VISIBLE TurretPlugin : public ModelPlugin{
    event::ConnectionPtr update_conn_;
public:
    TurretPlugin();
    ~TurretPlugin();

    void Load(physics::ModelPtr model, sdf::ElementPtr _sdf) override;
    void Reset() override;
    void OnUpdate();
    bool GetParam(sdf::ElementPtr sdf);
    void PointCallback(const geometry_msgs::Point& point_msg);

private: 
    physics::ModelPtr model_;
    physics::WorldPtr world_;
    common::Time last_time_;
    physics::LinkPtr link_;

    ros::NodeHandle nh_;
    ros::Subscriber point_sub_;

    std::string topic_name_;
    std::string yaw_joint_name_;
    std::string pitch_joint_name_;
    float yaw_p_;
    float pitch_p_;
    float yaw_target_;
    float pitch_target_;
};
} // end namespace gazebo