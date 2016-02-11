#include "ArmManager.h"
#include <controller_manager_msgs/SwitchController.h>

Joint::Joint(ros::NodeHandle& ph, int arm, int joint):
        goal(0)
{
    std::stringstream joint_topic;
    joint_topic << "arm_" << arm << "_joint_" << joint << "_position_controller/command";
    pos_pub = ph.advertise<std_msgs::Float64>(joint_topic.str(), 1, true);
}

int Joint::increment()
{
    goal += M_PI/180;
    return goal;
}

int Joint::decrement()
{
    goal -= M_PI/180;
    return goal;
}

void Joint::publish()
{
    std_msgs::Float64 msg;
    msg.data = goal;
    pos_pub.publish(msg);
}


ArmManager::ArmManager(ros::NodeHandle& ph):
        active_arm(1)
{
    serviceSwitchCtrl = ph.serviceClient<controller_manager_msgs::SwitchController>(
            "controller_manager/switch_controller"
    );

    for(int joint = 1; joint < 6; joint++)
    {
        left_arm.push_back(Joint(ph, 1, joint));
        right_arm.push_back(Joint(ph, 2, joint));
    }
}

int ArmManager::increment(int joint)
{
    if(active_arm == -1)
    {
        return left_arm[joint].increment();
    }
    else
    {
        return right_arm[joint].increment();
    }
}

int ArmManager::decrement(int joint)
{
    if(active_arm == -1)
    {
        return left_arm[joint].decrement();
    }
    else
    {
        return right_arm[joint].decrement();
    }
}

void ArmManager::publish()
{
    for(int joint = 0; joint < 5; joint++)
    {
        left_arm[joint].publish();
        right_arm[joint].publish();
    }
}

void ArmManager::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if(joy->axes[4] != 0)
    {
        active_arm = joy->axes[4];
    }

    if(joy->axes[5] == 1)
    {
        for(auto it = jointJoyBindings.begin(); it != jointJoyBindings.end(); it++)
        {
            if(joy->buttons[it->first])
            {
                increment(it->second);
            }
        }
    }

    if(joy->axes[5] == -1)
    {
        for(auto it = jointJoyBindings.begin(); it != jointJoyBindings.end(); it++)
        {
            if(joy->buttons[it->first])
            {
                decrement(it->second);
            }
        }
    }
}

bool ArmManager::handleKey(char key) {
    if (key == keyActiveArm)
        active_arm = -1*active_arm;
    else if (key == keySwitchController)
        switchController();
    else if (jointKeyBindings.count(key) > 0)
        active_joint = jointKeyBindings.at(key);
    else if (jointMoveKeyBindings.count(key) > 0) {
        if (jointMoveKeyBindings[key].first > 0)
            increment(active_joint);
        else if (jointMoveKeyBindings[key].first < 0)
            decrement(active_joint);
    }
    else
        return false;

    return true;
}

bool ArmManager::switchController() {
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
