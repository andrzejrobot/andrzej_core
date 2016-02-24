//
// Created by oles on 11.02.16.
//

#include "andrzej_control/joint_hardware_interface.h"

JointHardwareInterface::JointHardwareInterface(std::string resourceName, PCA9685Ptr pwmDriverPtr):
        name(resourceName), driverPtr(pwmDriverPtr)
{

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
    setGoal(cmd);
}

void JointHardwareInterface::read()
{
    pos = getPos();
}

void JointHardwareInterface::setGoal(double goal)
{

}

int JointHardwareInterface::getPos(void)
{
    return 0;
}
