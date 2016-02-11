#include <ros/ros.h>
#include <std_msgs/String.h>
#include <controller_manager/controller_manager.h>

#include "andrzej_control/andrzej_hardware_interface.h"

AndrzejHardwareInterface::AndrzejHardwareInterface()
{
    pwmDriverPtr = std::make_shared<PCA9685>(1,0x40);

    for ( int i = 1; i <= 2*JOINTS_PER_ARM; i++ )
    {
        std::stringstream ss;
        ss << "arm_" << i/JOINTS_PER_ARM + 1 << "_joint_" << i%JOINTS_PER_ARM;
        auto jointName = ss.str();

        jointNames.push_back(jointName);

        // connect and register the joint state interface
        hardware_interface::JointStateHandle stateHandle(jointName, &pos[0], &vel[0], &eff[0]);

        // connect and register the joint position interface
        hardware_interface::JointHandle posHandle(jointStateInterface.getHandle(jointName), &cmd[0]);
    }

    registerInterface(&jointStateInterface);
    registerInterface(&jointPosInterface);
}

void AndrzejHardwareInterface::write()
{

}

void AndrzejHardwareInterface::read()
{

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
