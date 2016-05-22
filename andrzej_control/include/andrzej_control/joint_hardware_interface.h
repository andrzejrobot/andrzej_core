//
// Created by oles on 11.02.16.
//

#ifndef ANDRZEJ_CONTROL_JOINT_HARDWARE_INTERFACE_H
#define ANDRZEJ_CONTROL_JOINT_HARDWARE_INTERFACE_H

#include <string>
#include <urdf/model.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include "PCA9685.h"

class HobbyServoHardwareInterface {
public:
    HobbyServoHardwareInterface() = default;
    HobbyServoHardwareInterface(std::string resourceName, PCA9685Ptr pwmDriverPtr, const urdf::Model& model);

    void write(void);
    void read(void);

    void registerHandle( hardware_interface::JointStateInterface& stateInterface,
                         hardware_interface::PositionJointInterface& posInterface,
                         joint_limits_interface::PositionJointSaturationInterface& limInterface);

private:
    static constexpr auto RAD_PER_PWM = 130.3478983923;
    static constexpr auto PWM_PER_RAD = 1/RAD_PER_PWM;
    static constexpr auto MAX_PWM = 500;
    static constexpr auto MIN_PWM = 180;

    std::string name = "";
    double cmd = 0, pos = 0, vel = 0, eff = 0;

    PCA9685Ptr driverPtr;
    int channel = 0;
    int offset = 0;
    int dir = 0;

    joint_limits_interface::JointLimits limits;

    void initServoParams(void);
    void initJointLimits(const urdf::Model& model);
    void initBasePosition(void);

    double pwmToRad(int pwm) const;
    int radToPwm(double rad) const;
};


#endif //ANDRZEJ_CONTROL_JOINT_HARDWARE_INTERFACE_H
