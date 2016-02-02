#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"
#include <vector>
#include <map>
#include <cmath> // use ros angles

class Joint
{
public:
  Joint(ros::NodeHandle& ph, int arm, int joint);

  int increment();
  int decrement();
  void publish();

private:
  float goal;

  ros::Publisher pos_pub;
};

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

typedef std::vector<Joint> Arm;

class ArmManager
{
public:
  ArmManager(ros::NodeHandle& ph);

  int increment(int joint);
  int decrement(int joint);
  void publish();
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

private:
  int active_arm;
  std::map<int, int> button_bindings;

  Arm left_arm, right_arm;
};

ArmManager::ArmManager(ros::NodeHandle& ph):
  active_arm(1)
{
  button_bindings[0] = 0;
  button_bindings[1] = 1;
  button_bindings[2] = 2;
  button_bindings[3] = 3;
  button_bindings[5] = 4;

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
    for(std::map<int, int>::iterator it = button_bindings.begin(); it != button_bindings.end(); it++)
    {
      if(joy->buttons[it->first])
      {
        increment(it->second);
      }
    }
  }

  if(joy->axes[5] == -1)
  {
    for(std::map<int, int>::iterator it = button_bindings.begin(); it != button_bindings.end(); it++)
    {
      if(joy->buttons[it->first])
      {
        decrement(it->second);
      }
    }
  }
}

class AndrzejTeleop
{
public:
  AndrzejTeleop();

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

AndrzejTeleop::AndrzejTeleop():
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

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &AndrzejTeleop::joyCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&AndrzejTeleop::publish, this));
}

void AndrzejTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  vel.angular.z = a_scale_*joy->axes[angular_];
  vel.linear.x = l_scale_x_*joy->axes[linear_x_];
  vel.linear.y = l_scale_y_*joy->axes[linear_y_];
  last_published_ = vel;
  deadman_pressed_ = joy->buttons[deadman_axis_];

  arm_mgr.joyCallback(joy);
}

void AndrzejTeleop::publish()
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
  AndrzejTeleop andrzej_teleop;

  ros::spin();
}
