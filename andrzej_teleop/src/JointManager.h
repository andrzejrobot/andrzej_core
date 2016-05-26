#ifndef ANDRZEJ_TELEOP_ARMMANAGER_H
#define ANDRZEJ_TELEOP_ARMMANAGER_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "Joint.h"

typedef std::vector<Joint> Joints;

class JointManager {
public:
    JointManager(ros::NodeHandle& ph);

    void increment();
    void decrement();
    void publish();

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    bool handleKey(char key);

    bool switchController();

    enum class ControlMode {
        JOINT,
        TRAJECTORY
    } ctrlMode = ControlMode::TRAJECTORY;

private:
    int activeJoint = 0;

    const std::map<int, int> jointJoyBindings = {
        {0, 0}, {1, 1}, {2, 2}, {3, 3}, {5, 4}, {6, 5}
    };
    const std::map<char, int> keyBindingsJointSelect = {
        {'1', 0}, {'2', 1}, {'3', 2}, {'4', 3}, {'5', 4}, {'6', 5}
    };
    std::map<char, std::pair<int,int>> keyBindingsJointMove = {
        {'a', {-1, 0}}, {'s', {1, 0}}, {'t', {0, -1}}, {'b', {0, 1}}
    };

    enum class JointSet {
        ARM1,
        ARM2,
        HEAD
    } activeJointSet = JointSet::ARM1;

    std::map<char, JointSet> keyBindingsActiveJointSet = {
            {'d', JointSet::ARM1}, {'f', JointSet::HEAD}, {'g', JointSet::ARM2}
    };
    const char keySwitchController = '`';
    const char keyToggleArms = '0';
    ros::ServiceClient serviceSwitchCtrl;

    Joints arm1, arm2, head;
    bool armsEnabled = false;
    void toggleArms();
    Joints& getActiveJointSet();
    Joint& getActiveJoint();
};


#endif //ANDRZEJ_TELEOP_ARMMANAGER_H
