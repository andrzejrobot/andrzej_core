//
// Created by oles on 11.02.16.
//

#include <ros/node_handle.h>
#include "andrzej_control/joint_hardware_interface.h"
#include <joint_limits_interface/joint_limits_urdf.h>

HobbyServoHardwareInterface::HobbyServoHardwareInterface(std::string resourceName,
                                                         PCA9685Ptr pwmDriverPtr,
                                                         const urdf::Model& model):
        name(resourceName), driverPtr(pwmDriverPtr)
{
    initServoParams();
    initJointLimits(model);
    initBasePosition();
}

void HobbyServoHardwareInterface::initServoParams(void)
{
    ros::NodeHandle nh;
    nh.getParam(name + "_servo/channel", channel);
    nh.getParam(name + "_servo/offset", offset);
    nh.getParam(name + "_servo/direction", dir);
}

void HobbyServoHardwareInterface::initJointLimits(const urdf::Model& model)
{
    auto joint_description = model.getJoint(name);
    if( !getJointLimits(joint_description, limits) )
        ROS_ERROR("Unable to load limits for %s", name.c_str());
    else
        ROS_INFO("Loaded limits for %s: %lf - %lf", name.c_str(), limits.min_position, limits.max_position);

    auto minPwm = radToPwm(limits.min_position);
    auto maxPwm = radToPwm(limits.max_position);
    if(minPwm < MIN_PWM || maxPwm > MAX_PWM)
        ROS_ERROR("WARNING: %s pwm limits %d - %d exceeds servo capabilities", minPwm, maxPwm);
}

void HobbyServoHardwareInterface::initBasePosition(void)
{
    ros::NodeHandle nh;
    nh.getParam(name + "/base_position", channel);
}

void HobbyServoHardwareInterface::registerHandle(hardware_interface::JointStateInterface& stateInterface,
                                                 hardware_interface::PositionJointInterface& posInterface,
                                                 joint_limits_interface::PositionJointSaturationInterface& limInterface)
{
    hardware_interface::JointStateHandle stateHandle(name, &pos, &vel, &eff);
    stateInterface.registerHandle(stateHandle);

    hardware_interface::JointHandle posHandle(stateInterface.getHandle(name), &cmd);
    posInterface.registerHandle(posHandle);

    joint_limits_interface::PositionJointSaturationHandle limHandle(posInterface.getHandle(name), limits);
    limInterface.registerHandle(limHandle);
}

void HobbyServoHardwareInterface::write()
{
    double pwm = radToPwm(cmd);
    driverPtr->setPWM( channel, static_cast<int>(pwm));
}

void HobbyServoHardwareInterface::read()
{
    int pwm = driverPtr->getPWM(channel);
    pos = pwmToRad(pwm);
}

double HobbyServoHardwareInterface::pwmToRad(int pwm) const
{
    return dir*(pwm - offset)*PWM_PER_RAD;
}

int HobbyServoHardwareInterface::radToPwm(double rad) const
{
    return dir*RAD_PER_PWM*rad + offset;
}
