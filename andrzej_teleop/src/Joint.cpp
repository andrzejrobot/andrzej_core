//
// Created by oles on 22.05.16.
//

#include "Joint.h"

Joint::Joint(ros::NodeHandle& ph, std::string name)
{
    auto joint_topic = name + "_position_controller/command";
    pos_pub = ph.advertise<std_msgs::Float64>(joint_topic, 1, true);
}

void Joint::increment()
{
    if (goal <= limitMax)
        goal += STEP;
}

void Joint::decrement()
{
    if (goal >= limitMin)
        goal -= STEP;
}

void Joint::publish()
{
    std_msgs::Float64 msg;
    msg.data = goal;
    pos_pub.publish(msg);
}