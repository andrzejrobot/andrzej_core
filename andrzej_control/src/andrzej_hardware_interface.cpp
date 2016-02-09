#include <ros/ros.h>
#include <std_msgs/String.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>


class Andrzej : public hardware_interface::RobotHW
{
public:
  Andrzej() 
  { 
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_a("A", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_a);

    hardware_interface::JointStateHandle state_handle_b("B", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_b);

    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("A"), &cmd[0]);
    jnt_pos_interface.registerHandle(pos_handle_a);

    hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("B"), &cmd[1]);
    jnt_pos_interface.registerHandle(pos_handle_b);

    registerInterface(&jnt_pos_interface);
  }
  
  void write()
  {
  
  }
  
  void read()
  {
  
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "andrzej_hardware_interface");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);

  Andrzej robot;
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
