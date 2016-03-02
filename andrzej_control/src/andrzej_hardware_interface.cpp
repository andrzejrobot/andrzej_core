#include <ros/ros.h>
#include <std_msgs/String.h>
#include <controller_manager/controller_manager.h>

#include "andrzej_control/andrzej_hardware_interface.h"

AndrzejHardwareInterface::AndrzejHardwareInterface()
{
    pwmDriverPtr = std::make_shared<PCA9685>(1,0x40);

    for ( uint i = 0; i < JOINTS_PER_ARM; i++ )
    {
        std::stringstream ss;
        ss << "arm_" << 1 << "_joint_" << i+1;
        arm_1[i] = JointHardwareInterface(ss.str(), pwmDriverPtr);
        arm_1[i].registerHandle(jointStateInterface, jointPosInterface);
        ss.str(std::string());
        ss << "arm_" << 2 << "_joint_" << i+1;
        arm_2[i] = JointHardwareInterface(ss.str(), pwmDriverPtr);
        arm_2[i].registerHandle(jointStateInterface, jointPosInterface);
    }
    registerInterface(&jointStateInterface);
    registerInterface(&jointPosInterface);
}

void AndrzejHardwareInterface::write()
{
    for (auto &joint : arm_1)
        joint.write();

    for (auto &joint : arm_2)
        joint.write();
}

void AndrzejHardwareInterface::read()
{
    for (auto &joint : arm_1)
        joint.read();

    for (auto &joint : arm_2)
        joint.read();
}

ros::Time AndrzejHardwareInterface::get_time()
{
    prev_update_time = curr_update_time;
    curr_update_time = ros::Time::now();
    return curr_update_time;
}

ros::Duration AndrzejHardwareInterface::get_period()
{
    return curr_update_time - prev_update_time;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "andrzej_hardware_interface");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    AndrzejHardwareInterface robot;
    controller_manager::ControllerManager cm(&robot, nh);

    while (ros::ok())
    {
        robot.read();
        cm.update(robot.get_time(), robot.get_period());
        robot.write();
        loop_rate.sleep();
    }

    return 0;
}
