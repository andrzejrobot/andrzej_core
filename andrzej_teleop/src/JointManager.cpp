#include "JointManager.h"
#include <controller_manager_msgs/SwitchController.h>
#include <std_srvs/Empty.h>

JointManager::JointManager(ros::NodeHandle& ph)
{
    serviceSwitchCtrl = ph.serviceClient<controller_manager_msgs::SwitchController>(
            "controller_manager/switch_controller"
    );

    for(int joint = 1; joint < 6; joint++) {
        std::stringstream ss;
        ss << "arm_" << 1 << "_joint_" << joint;
        arm1.push_back(Joint(ph, ss.str()));
        ss.str(std::string());
        ss << "arm_" << 2 << "_joint_" << joint;
        arm2.push_back(Joint(ph, ss.str()));
    }
    arm1.push_back(Joint(ph, "arm_1_gripper_joint"));
    arm2.push_back(Joint(ph, "arm_2_gripper_joint"));

    head.push_back(Joint(ph, "head_pan_joint"));
    head.push_back(Joint(ph, "head_tilt_joint"));
}

void JointManager::increment()
{
    return getActiveJoint().increment();
}

void JointManager::decrement()
{
    return getActiveJoint().decrement();
}

void JointManager::publish()
{
    getActiveJoint().publish();
}

void JointManager::toggleArms()
{
    static auto srv = std_srvs::Empty();
    if(armsEnabled)
        ros::service::call("andrzej_hw/disableArms", srv);
    else
        ros::service::call("andrzej_hw/enableArms", srv);
    armsEnabled = !armsEnabled;
}

void JointManager::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if(joy->axes[4] != 0)
        activeJointSet = (joy->axes[4] < 0) ? JointSet::ARM1 : JointSet::ARM2;
    else if(joy->buttons[8])
        activeJointSet = JointSet::HEAD;

    if(joy->axes[5] != 0)
        for(auto it = jointJoyBindings.begin(); it != jointJoyBindings.end(); it++)
            if (joy->buttons[it->first]) {
                activeJoint = it->second;
                (joy->axes[5] > 0) ? increment() : decrement();
            }

    if(joy->buttons[9])
        toggleArms();
}

bool JointManager::handleKey(char key) {
    if (keyBindingsActiveJointSet.count(key))
        activeJointSet = keyBindingsActiveJointSet.at(key);
    else if(key == keySwitchController)
        switchController();
    else if(key == keyToggleArms)
        toggleArms();
    else if (keyBindingsJointSelect.count(key) > 0)
        activeJoint = keyBindingsJointSelect.at(key);
    else if (keyBindingsJointMove.count(key) > 0) {
        if (keyBindingsJointMove[key].first > 0)
            increment();
        else if (keyBindingsJointMove[key].first < 0)
            decrement();
    }
    else
        return false;

    return true;
}

bool JointManager::switchController() {
    controller_manager_msgs::SwitchControllerRequest req;
    req.stop_controllers = {
            "arm_1_joint_1_position_controller",
            "arm_1_joint_2_position_controller",
            "arm_1_joint_3_position_controller",
            "arm_1_joint_4_position_controller",
            "arm_1_joint_5_position_controller",
            "arm_1_gripper_position_controller",
            "arm_2_joint_1_position_controller",
            "arm_2_joint_2_position_controller",
            "arm_2_joint_3_position_controller",
            "arm_2_joint_4_position_controller",
            "arm_2_joint_5_position_controller",
            "arm_2_gripper_position_controller"
    };
    req.start_controllers = {
            "arm_1_controller",
            "arm_1_gripper_controller",
            "arm_2_controller",
            "arm_2_gripper_controller"
    };

    req.strictness = req.STRICT;

    if(ctrlMode == ControlMode::TRAJECTORY) {
        ctrlMode = ControlMode::JOINT;
        auto tmp = req.start_controllers;
        req.start_controllers = req.stop_controllers;
        req.stop_controllers = tmp;
    }
    else
        ctrlMode = ControlMode::TRAJECTORY;

    controller_manager_msgs::SwitchControllerResponse resp;
    bool ret = serviceSwitchCtrl.call(req, resp);

    std::stringstream ss;
    ss << " stop ";
    for (auto i = req.stop_controllers.begin(); i != req.stop_controllers.end(); ++i) {
        ss << *i << ' ';
    }
    ss << "start ";
    for (auto i = req.start_controllers.begin(); i != req.start_controllers.end(); ++i) {
        ss << *i << ' ';
    }
    ss << "output " << ret;
    ROS_INFO_STREAM( "Service call " << serviceSwitchCtrl.getService() << ss.str() );

    return ret;
}

Joints& JointManager::getActiveJointSet() {
    if(activeJointSet == JointSet::ARM1)
        return arm1;
    else if(activeJointSet == JointSet::ARM2)
        return arm2;
    else
        return head;
}

Joint& JointManager::getActiveJoint() {
    if (activeJoint >= getActiveJointSet().size())
        activeJoint = getActiveJointSet().size() - 1;
    return getActiveJointSet().at(activeJoint);
}