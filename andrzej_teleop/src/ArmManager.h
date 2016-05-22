#ifndef ANDRZEJ_TELEOP_ARMMANAGER_H
#define ANDRZEJ_TELEOP_ARMMANAGER_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "Joint.h"

typedef std::vector<Joint> Arm;

class ArmManager {
public:
    ArmManager(ros::NodeHandle& ph);

    void increment(int joint);
    void decrement(int joint);
    void publish();

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    bool handleKey(char key);

    bool switchController();

    enum class ControlMode {
        JOINT,
        TRAJECTORY
    } ctrlMode = ControlMode::TRAJECTORY;

private:
    int active_arm = 1;
    int active_joint = 0;

    const std::map<int, int> jointJoyBindings = {
        {0, 0}, {1, 1}, {2, 2}, {3, 3}, {5, 4},
    };
    const std::map<char, int> jointKeyBindings = {
            {'1', 0}, {'2', 1}, {'3', 2}, {'4', 3}, {'5', 4}, {'6', 5}
    };
    std::map<char, std::pair<int,int>> jointMoveKeyBindings = {
        {'a', {-1, 0}}, {'s', {0, -1}}, {'d', {0, 1}}, {'f', {1, 0}}
    };
    const char keyActiveArm = '\t';
    const char keySwitchController = '`';
    ros::ServiceClient serviceSwitchCtrl;

    Arm left_arm, right_arm;
};


#endif //ANDRZEJ_TELEOP_ARMMANAGER_H
