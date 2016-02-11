#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include "ArmManager.h"

class AndrzejTeleopKey
{
public:
    AndrzejTeleopKey();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void publish();

    ros::NodeHandle ph_, nh_;

    int linear_x_, linear_y_, angular_, deadman_axis_;
    double l_scale_x_, l_scale_y_, a_scale_;
    ArmManager arm_mgr;
    ros::Publisher vel_pub_;
    std::vector<Arm> joint_pub_;
    ros::Subscriber joy_sub_;

    geometry_msgs::Twist last_published_;
    boost::mutex publish_mutex_;
    bool deadman_pressed_;
    bool zero_twist_published_;
    ros::Timer timer_;

};

AndrzejTeleopKey::AndrzejTeleopKey():
        ph_("~"),
        arm_mgr(ph_),
        linear_x_(1),
        linear_y_(0),
        angular_(2),
        deadman_axis_(4),
        l_scale_x_(0.3),
        l_scale_y_(0.3),
        a_scale_(0.9)
{
    ph_.param("axis_linear_x", linear_x_, linear_x_);
    ph_.param("axis_linear_y", linear_y_, linear_y_);
    ph_.param("axis_angular", angular_, angular_);
    ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
    ph_.param("scale_angular", a_scale_, a_scale_);
    ph_.param("scale_linear_x", l_scale_x_, l_scale_x_);
    ph_.param("scale_linear_y", l_scale_y_, l_scale_y_);

    deadman_pressed_ = false;
    zero_twist_published_ = false;

    vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &AndrzejTeleopKey::joyCallback, this);

    timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&AndrzejTeleopKey::publish, this));
}

void AndrzejTeleopKey::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist vel;
    vel.angular.z = a_scale_*joy->axes[angular_];
    vel.linear.x = l_scale_x_*joy->axes[linear_x_];
    vel.linear.y = l_scale_y_*joy->axes[linear_y_];
    last_published_ = vel;
    deadman_pressed_ = joy->buttons[deadman_axis_];

    arm_mgr.joyCallback(joy);
}

void AndrzejTeleopKey::publish()
{
    boost::mutex::scoped_lock lock(publish_mutex_);

    if (deadman_pressed_)
    {
        vel_pub_.publish(last_published_);
        zero_twist_published_=false;
    }
    else if(!deadman_pressed_ && !zero_twist_published_)
    {
        vel_pub_.publish(*new geometry_msgs::Twist());
        zero_twist_published_=true;
    }

    arm_mgr.publish();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "andrzej_teleop");
    AndrzejTeleopKey andrzej_teleop;

    ros::spin();
}
