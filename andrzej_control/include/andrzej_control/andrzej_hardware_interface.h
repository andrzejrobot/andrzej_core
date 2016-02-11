#include <memory>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "PCA9685.h"

class AndrzejHardwareInterface : public hardware_interface::RobotHW
{
public:
    AndrzejHardwareInterface();

    void write(void);
    void read(void);

private:
    static const int JOINTS_PER_ARM = 5;

    hardware_interface::JointStateInterface jointStateInterface;
    hardware_interface::PositionJointInterface jointPosInterface;

    std::array<std::string, JOINTS_PER_ARM> jointNames;
    std::array<double, JOINTS_PER_ARM> jointCmd, jointPos, jointVel, jointEff;

    std::shared_ptr<PCA9685> pwmDriverPtr;
};