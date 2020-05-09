#pragma once

#include<ros/ros.h>
#include<geometry_msgs/Point.h>
#include<sensor_msgs/JointState.h>
#include<string>
#include<cmath>
#include<limits>

struct Angle4D{
    float angle1;
    float angle2;
    float angle3;
    float angle4;
};

class ArmMock{
public:
    ArmMock(float l1, float l2, float l3, float l4, float l5){
        length1_ = l1;
        length2_ = l2;
        length3_ = l3;
        length4_ = l4;
        length5_ = l5;
    }

    void setAngle(float angle1, float angle2, float angle3, float angle4){
        arm1_angle_ = angle1;
        arm2_angle_ = angle2;
        arm3_angle_ = angle3;
        arm4_angle_ = angle4;
    }

    void setAngle(Angle4D angles){
        arm1_angle_ = angles.angle1;
        arm2_angle_ = angles.angle2;
        arm3_angle_ = angles.angle3;
        arm4_angle_ = angles.angle4;
    }

    sensor_msgs::JointState getJointState(){
        sensor_msgs::JointState output;
        output.header.stamp = ros::Time::now();
        output.name.resize(4);
        output.name[0] = arm1_joint_name_;
        output.name[1] = arm2_joint_name_;
        output.name[2] = arm3_joint_name_;
        output.name[3] = arm4_joint_name_;
        output.position.resize(4);
        output.position[0] = arm1_angle_;
        output.position[1] = arm2_angle_;
        output.position[2] = arm3_angle_;
        output.position[3] = arm4_angle_;
        return output;
    }

    geometry_msgs::Point getTargetPoint(){
        geometry_msgs::Point output;
        float l = length3_ * sin(arm2_angle_) + length4_ * sin(arm2_angle_ + arm3_angle_) + length5_ * sin(arm2_angle_ + arm3_angle_ + arm4_angle_);
        float z = length1_ + length2_ +length3_ * cos(arm2_angle_) + length4_ * cos(arm2_angle_ + arm3_angle_) + length5_ * cos(arm2_angle_ + arm3_angle_ + arm4_angle_);
        output.x = l * cos(arm1_angle_);
        output.y = l * sin(arm1_angle_);
        output.z = z;
        return output;
    }

    float length1_ = 0.1;
    float length2_ = 0.1;
    float length3_ = 0.1;
    float length4_ = 0.1;
    float length5_ = 0.1;

    float arm1_angle_ = 0.0;
    float arm2_angle_ = 0.0;
    float arm3_angle_ = 0.0;
    float arm4_angle_ = 0.0;

    std::string arm1_joint_name_="arm1_joint";
    std::string arm2_joint_name_="arm2_joint";
    std::string arm3_joint_name_="arm3_joint";
    std::string arm4_joint_name_="arm4_joint";
};

class ArmSolver{
public:
    ArmSolver(float l1, float l2, float l3, float l4, float l5){
        length1_ = l1;
        length2_ = l2;
        length3_ = l3;
        length4_ = l4;
        length5_ = l5;
    }

    bool solve(geometry_msgs::Point point_msg, float target_angle, Angle4D& output){
        output.angle1 = getDirection(point_msg);
        float ik_l, ik_z;
        getLZ(point_msg, target_angle, ik_l, ik_z);
        output.angle2 = getAngle1(ik_l, ik_z);
        output.angle3 = getAngle2(ik_l, ik_z);
        output.angle4 = target_angle - (output.angle2 + output.angle3);
        if(checkAngleValid(output))return true;
        else return false;
    }

    float getDirection(geometry_msgs::Point point_msg){
        return atan2(point_msg.y, point_msg.x);
    }

    void getLZ(geometry_msgs::Point point_msg, float target_angle, float& ik_l, float& ik_z){
        float raw_l = sqrt(point_msg.x * point_msg.x + point_msg.y * point_msg.y);
        float raw_z = point_msg.z;
        ik_l = raw_l - length5_ * sin(target_angle);
        ik_z = raw_z - length1_ - length2_ - length5_ * cos(target_angle);
    }

    float getAngle1(float ik_l, float ik_z){
        float v = sqrt(ik_l * ik_l + ik_z * ik_z);
        return M_PI / 2.0 - getAngle(length3_, v, length4_) - atan2(ik_z, ik_l);
    }

    float getAngle2(float ik_l, float ik_z){
        float v = sqrt(ik_l * ik_l + ik_z * ik_z);
        return M_PI - getAngle(length3_, length4_, v);
    }

    float getAngle(float a, float b, float c){
        float cos_n =  a * a + b * b - c * c;
        float cos_d = 2.0 * a * b;
        return acos(cos_n / cos_d);
    }

    bool checkAngleValid(Angle4D angles){
        if(std::isnan(angles.angle1))return false;
        if(std::isnan(angles.angle2))return false;
        if(std::isnan(angles.angle3))return false;
        if(std::isnan(angles.angle4))return false;
        return true;
    }

    float length1_ = 0.1;
    float length2_ = 0.1;
    float length3_ = 0.1;
    float length4_ = 0.1;
    float length5_ = 0.1;
};

class ArmSmooth{
public:
    ArmSmooth(void){}

    void setTargetAngles(Angle4D angles){
        start_angles_ = current_angles_;
        end_angles_ = angles;
    }

    void setCurrentAngles(Angle4D angles){
        current_angles_ = angles;
    }

    Angle4D output(float value){
        if(value < 0.0) value = 0.0;
        if(value > 1.0) value = 1.0;
        Angle4D output;
        output.angle1 = (1.0 - value) * start_angles_.angle1 + value * end_angles_.angle1;
        output.angle2 = (1.0 - value) * start_angles_.angle2 + value * end_angles_.angle2;
        output.angle3 = (1.0 - value) * start_angles_.angle3 + value * end_angles_.angle3;
        output.angle4 = (1.0 - value) * start_angles_.angle4 + value * end_angles_.angle4;
        current_angles_ = output;
        return output;
    }

    Angle4D start_angles_;
    Angle4D end_angles_;
    Angle4D current_angles_;    
};