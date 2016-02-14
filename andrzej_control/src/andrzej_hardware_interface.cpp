#include <ros/ros.h>
#include <std_msgs/String.h>
#include <controller_manager/controller_manager.h>

#include "andrzej_control/andrzej_hardware_interface.h"

AndrzejHardwareInterface::AndrzejHardwareInterface()
{
    pwmDriverPtr = std::make_shared<PCA9685>(1,0x40);

    for ( int i = 0; i < JOINTS_PER_ARM; i++ )
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "andrzej_hardware_interface");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10);

    AndrzejHardwareInterface robot;
    controller_manager::ControllerManager cm(&robot);

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();
        chatter_pub.publish(msg);

        robot.read();
        cm.update(ros::Time::now(), ros::Duration(0.1));
        robot.write();

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
