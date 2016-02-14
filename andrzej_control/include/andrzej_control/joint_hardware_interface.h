//
// Created by oles on 11.02.16.
//

#ifndef ANDRZEJ_CONTROL_JOINT_HARDWARE_INTERFACE_H
#define ANDRZEJ_CONTROL_JOINT_HARDWARE_INTERFACE_H

#include <string>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

#include "PCA9685.h"

class JointHardwareInterface {
public:
    JointHardwareInterface() = default;
    JointHardwareInterface(std::string resourceName, PCA9685Ptr pwmDriverPtr);

    void write(void);
    void read(void);

    void registerHandle( hardware_interface::JointStateInterface& stateInterface,
                         hardware_interface::PositionJointInterface& posInterface);

    void setGoal(double goal);
    int getPos(void);

private:
    int PWMtoRADratio = 1;
    int PWMtoRADoffset = 0;
    std::string name = "";

    double cmd, pos, vel, eff;

    PCA9685Ptr driverPtr;
};


#endif //ANDRZEJ_CONTROL_JOINT_HARDWARE_INTERFACE_H
