//
// Created by oles on 11.02.16.
//

#ifndef ANDRZEJ_CONTROL_JOINT_HARDWARE_INTERFACE_H
#define ANDRZEJ_CONTROL_JOINT_HARDWARE_INTERFACE_H

#include <string>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include "PCA9685.h"

class JointHardwareInterface {
public:
    JointHardwareInterface() = default;
    JointHardwareInterface(std::string resourceName, PCA9685Ptr pwmDriverPtr);

    void write(void);
    void read(void);

    void registerHandle( hardware_interface::JointStateInterface& stateInterface,
                         hardware_interface::PositionJointInterface& posInterface,
                         joint_limits_interface::PositionJointSaturationInterface& limInterface);

private:
    std::string name = "";

    double cmd, pos, vel, eff;

    PCA9685Ptr driverPtr;
    int channel = 0;
    int offset = 0;
    float ratio = 1.f;

    joint_limits_interface::JointLimits limits;
};


#endif //ANDRZEJ_CONTROL_JOINT_HARDWARE_INTERFACE_H
