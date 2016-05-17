#include <functional>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include "andrzej_control/andrzej_hardware_interface.h"

auto arm1Enabled = false;
auto arm2Enabled = false;
auto headEnabled = true;

AndrzejHardwareInterface::AndrzejHardwareInterface()
{
    ros::NodeHandle nh;
    std::string robot_description;
    if ( !nh.getParam("robot_description", robot_description) )
        ROS_ERROR("Unable to load robot description");
    //else
    //    ROS_INFO("%s", robot_description.c_str());

    urdf::Model model;
    if( !model.initString(robot_description) )
        ROS_ERROR("Unable to initialize urdf model");

    pwmDriverPtr = std::make_shared<PCA9685>(1,0x40);

    for ( uint i = 0; i < JOINTS_PER_ARM; i++ )
    {
        std::stringstream ss;
        ss << "arm_" << 1 << "_joint_" << i+1;
        arm_1[i] = HobbyServoHardwareInterface(ss.str(), pwmDriverPtr, model);
        arm_1[i].registerHandle(jointStateInterface, jointPosInterface, jointLimInterface);
        ss.str(std::string());
        ss << "arm_" << 2 << "_joint_" << i+1;
        arm_2[i] = HobbyServoHardwareInterface(ss.str(), pwmDriverPtr, model);
        arm_2[i].registerHandle(jointStateInterface, jointPosInterface, jointLimInterface);
    }

    headPan = HobbyServoHardwareInterface("head_pan_joint", pwmDriverPtr, model);
    headPan.registerHandle(jointStateInterface, jointPosInterface, jointLimInterface);
    headTilt = HobbyServoHardwareInterface("head_tilt_joint", pwmDriverPtr, model);
    headTilt.registerHandle(jointStateInterface, jointPosInterface, jointLimInterface);

    registerInterface(&jointStateInterface);
    registerInterface(&jointPosInterface);
    registerInterface(&jointLimInterface);
}

void AndrzejHardwareInterface::write()
{
    jointLimInterface.enforceLimits(get_period());

    if (arm1Enabled)
        for (auto &joint : arm_1)
            joint.write();

    if (arm2Enabled)
        for (auto &joint : arm_2)
            joint.write();

    if (headEnabled) {
        headPan.write();
        headTilt.write();
    }
}

void AndrzejHardwareInterface::read()
{
    for (auto &joint : arm_1)
        joint.read();

    for (auto &joint : arm_2)
        joint.read();

    headPan.read();
    headTilt.read();
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

    nh.param("enableArm1", arm1Enabled, arm1Enabled);
    nh.param("enableArm2", arm2Enabled, arm2Enabled);
    nh.param("enableHead", headEnabled, headEnabled);

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
