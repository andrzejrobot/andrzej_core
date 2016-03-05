//
// Created by oles on 11.02.16.
//

#include <ros/node_handle.h>
#include "andrzej_control/joint_hardware_interface.h"
#include <joint_limits_interface/joint_limits_urdf.h>

JointHardwareInterface::JointHardwareInterface(std::string resourceName,
                                               PCA9685Ptr pwmDriverPtr,
                                               const urdf::Model& model):
        name(resourceName), driverPtr(pwmDriverPtr)
{
    ros::NodeHandle nh;
    std::string servo_type = "turnigy";
    nh.getParam(name + "_servo/type", servo_type);
    nh.getParam(name + "_servo/channel", channel);
    nh.getParam(name + "_servo/offset", offset);
    nh.getParam(servo_type + "_ratio", ratio);

    auto joint_description = model.getJoint(name);
    if( !getJointLimits(joint_description, limits) )
        ROS_ERROR("Unable to load limits for %s", name.c_str());
    else
        ROS_INFO("Loaded limits for %s, min: %lf, max: %lf", name.c_str(), limits.min_position, limits.max_position);
}

void JointHardwareInterface::registerHandle( hardware_interface::JointStateInterface& stateInterface,
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

void JointHardwareInterface::write()
{
    float pwm = ratio*cmd + offset;
    driverPtr->setPWM( channel, static_cast<int>(pwm));
}

void JointHardwareInterface::read()
{
    int pwm = driverPtr->getPWM(channel);
    pos = (pwm - offset)/ratio;
}