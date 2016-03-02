//
// Created by oles on 11.02.16.
//

#include <ros/node_handle.h>
#include "andrzej_control/joint_hardware_interface.h"

JointHardwareInterface::JointHardwareInterface(std::string resourceName, PCA9685Ptr pwmDriverPtr):
        name(resourceName), driverPtr(pwmDriverPtr)
{
    ros::NodeHandle nh;
    std::string servo_type = "turnigy";
    nh.getParam(name + "_servo/type", servo_type);
    nh.getParam(name + "_servo/channel", channel);
    nh.getParam(name + "_servo/offset", offset);
    nh.getParam(servo_type + "_ratio", ratio);
}

void JointHardwareInterface::registerHandle( hardware_interface::JointStateInterface& stateInterface,
                                             hardware_interface::PositionJointInterface& posInterface)
{
    hardware_interface::JointStateHandle stateHandle(name, &pos, &vel, &eff);
    stateInterface.registerHandle(stateHandle);

    hardware_interface::JointHandle posHandle(stateInterface.getHandle(name), &cmd);
    posInterface.registerHandle(posHandle);
}

void JointHardwareInterface::write()
{
    float pwm = ratio*cmd + offset;
    driverPtr->setPWM(channel, static_cast<uint8_t>(pwm));
}

void JointHardwareInterface::read()
{
    int pwm = driverPtr->getPWM(channel);
    pos = (pwm - offset)/ratio;
}