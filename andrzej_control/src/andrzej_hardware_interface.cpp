#include <functional>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include "andrzej_control/andrzej_hardware_interface.h"

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
    gripperLeft = HobbyServoHardwareInterface("arm_1_gripper_joint", pwmDriverPtr, model);
    gripperLeft.registerHandle(jointStateInterface, jointPosInterface, jointLimInterface);
    gripperRight = HobbyServoHardwareInterface("arm_2_gripper_joint", pwmDriverPtr, model);
    gripperRight.registerHandle(jointStateInterface, jointPosInterface, jointLimInterface);

    headPan = HobbyServoHardwareInterface("head_pan_joint", pwmDriverPtr, model);
    headPan.registerHandle(jointStateInterface, jointPosInterface, jointLimInterface);
    headPan.enable();
    headTilt = HobbyServoHardwareInterface("head_tilt_joint", pwmDriverPtr, model);
    headTilt.registerHandle(jointStateInterface, jointPosInterface, jointLimInterface);
    headTilt.enable();

    registerInterface(&jointStateInterface);
    registerInterface(&jointPosInterface);
    registerInterface(&jointLimInterface);

    armEnabler = nh.advertiseService("andrzej_hw/enableArms", &AndrzejHardwareInterface::enableArms, this);
    armDisabler = nh.advertiseService("andrzej_hw/disableArms", &AndrzejHardwareInterface::disableArms, this);
}

void AndrzejHardwareInterface::write()
{
    jointLimInterface.enforceLimits(get_period());

    for (auto& joint : arm_1)
        joint.write();
    gripperLeft.write();

    for (auto& joint : arm_2)
        joint.write();
    gripperRight.write();

    headPan.write();
    headTilt.write();
}

void AndrzejHardwareInterface::read()
{
    for (auto &joint : arm_1)
        joint.read();
    gripperLeft.read();

    for (auto &joint : arm_2)
        joint.read();
    gripperRight.read();

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


bool AndrzejHardwareInterface::enableArms(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    for(auto& joint : arm_1)
        joint.enable();
    gripperLeft.enable();
    for(auto& joint : arm_2)
        joint.enable();
    gripperRight.enable();
    return true;
}

bool AndrzejHardwareInterface::disableArms(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    for(auto& joint : arm_1)
        joint.disable();
    gripperLeft.disable();
    for(auto& joint : arm_2)
        joint.disable();
    gripperRight.disable();
    return true;
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
