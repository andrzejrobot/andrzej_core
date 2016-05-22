#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <boost/thread/thread.hpp>
#include "JointManager.h"

typedef std::map<char, std::pair<float,float>> KeyBindings;
KeyBindings moveBindings = {
        {'i', { 1,  0}},
        {'o', { 1, -1}},
        {'j', { 0,  1}},
        {'l', { 0, -1}},
        {'u', { 1,  1}},
        {',', {-1,  0}},
        {'.', {-1,  1}},
        {'m', {-1, -1}}
};
KeyBindings speedBindings = {
        {'q', {1.1, 1.1}},
        {'z', {0.9, 0.9}},
        {'w', {1.1, 1.0}},
        {'x', {0.9, 1.0}},
        {'e', {1.0, 1.1}},
        {'c', {1.0, 0.9}}
};

class AndrzejTeleopKey
{
public:
    AndrzejTeleopKey();

    void run();
private:
    ros::NodeHandle ph_, nh_;

    JointManager arm_mgr;
    ros::Publisher vel_pub_;
};

AndrzejTeleopKey::AndrzejTeleopKey():
        ph_("~"),
        arm_mgr(ph_)
{
    vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
}

void AndrzejTeleopKey::run() {
    char key = 0;

    float speed = 0.2, turn = 1;

    float x = 0;
    float th = 0;
    int count = 0;
    float target_speed = 0;
    float target_turn = 0;
    float control_speed = 0;
    float control_turn = 0;

    system("stty raw");
    while(true)
    {
        std::cin.get(key);
        std::cout << '\r' << std::flush;

        if (arm_mgr.handleKey(key)) {
            arm_mgr.publish();
            continue;
        }
        else if(moveBindings.count(key) > 0) {
            x = moveBindings[key].first;
            th = moveBindings[key].second;
            count = 0;
        }
        else if (speedBindings.count(key) > 0) {
            speed = speed * speedBindings[key].first;
            turn = turn * speedBindings[key].second;
            count = 0;
        }
        else if (key == ' ' || key == 'k') {
            x = 0;
            th = 0;
            control_speed = 0;
            control_turn = 0;
        }
        else {
            count = count + 1;
            if (count > 4) {
                x = 0;
                th = 0;
            }

            if (key == '\x03')
                break;
        }

        target_speed = speed * x;
        target_turn = turn * th;

        std::cout << target_speed;
        if (target_speed > control_speed)
            control_speed = std::min(target_speed, control_speed + 0.02f);
        else if (target_speed < control_speed)
            control_speed = std::max(target_speed, control_speed - 0.02f);
        else
            control_speed = target_speed;

        if (target_turn > control_turn)
            control_turn = std::min(target_turn, control_turn + 0.1f);
        else if (target_turn < control_turn)
            control_turn = std::max(target_turn, control_turn - 0.1f);
        else
            control_turn = target_turn;

        std::cout << control_speed;
        geometry_msgs::Twist twist;
        twist.linear.x = control_speed;
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = control_turn;
        vel_pub_.publish(twist);
    }
    system("stty cooked");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "andrzej_teleop");
    AndrzejTeleopKey andrzej_teleop;

    andrzej_teleop.run();

    return 0;
}
