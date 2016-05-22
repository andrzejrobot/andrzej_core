//
// Created by oles on 22.05.16.
//

#ifndef ANDRZEJ_TELEOP_JOINT_H
#define ANDRZEJ_TELEOP_JOINT_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <boost/math/constants/constants.hpp>

class Joint
{
public:
    Joint(ros::NodeHandle& ph, std::string name);

    void increment();
    void decrement();
    void publish();

private:
    static constexpr auto STEP = boost::math::constants::pi<float>()/180;

    float goal = 0.0f;
    float limitMin = -boost::math::constants::pi<float>()/2;
    float limitMax = boost::math::constants::pi<float>()/2;

    ros::Publisher pos_pub;
};


#endif //ANDRZEJ_TELEOP_JOINT_H
