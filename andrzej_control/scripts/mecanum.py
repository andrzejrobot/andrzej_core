#!/usr/bin/env python

import math
import rospy
import wiringpi

from geometry_msgs.msg import Twist

GPIO_OUT = 1

PWM1 = 25
PWM2 = 11
PWM3 = 7
PWM4 = 8

DIR1 = 23
DIR2 = 24
DIR3 = 27
DIR4 = 22

PWM_PINS = [PWM1, PWM2, PWM3, PWM4]
DIR_PINS = [DIR1, DIR2, DIR3, DIR4]

PWM_RANGE = 100

class Wheel:
    def __init__(self, pwm_pin, dir_pin):
        self._pwm_pin = pwm_pin
        self._dir_pin = dir_pin
        wiringpi.pinMode(self._dir_pin, wiringpi.OUTPUT)
        wiringpi.pinMode(self._pwm_pin, wiringpi.OUTPUT)
        wiringpi.softPwmCreate(self._pwm_pin, 0, PWM_RANGE)
        wiringpi.softPwmWrite(self._pwm_pin, 0)

    def setPWM(self, pwm):
        wiringpi.softPwmWrite(self._pwm_pin, int(PWM_RANGE*pwm))

    def setDir(self, direction):
        wiringpi.digitalWrite(self._dir_pin, int(direction))


class MecanumChassis:
    def __init__(self, pwm_pins, dir_pins):
        self._wheels = [Wheel(p, d) for p, d in zip(pwm_pins, dir_pins)]

    def driveFK(self, pwms, dirs):
        for wheel, p, d in zip(self._wheels, pwms, dirs):
            wheel.setPWM(p)
            wheel.setDir(d)

    def driveIK(self, twist):
        x = twist.linear.x
        y = twist.linear.y
        w = twist.angular.z

        a = math.atan2(y, x)
        v = math.sqrt(y*y + x*x)

        speeds = [ v * math.sin(a + math.pi/4) + w,
                   v * math.cos(a + math.pi/4) - w,
                   v * math.cos(a + math.pi/4) + w,
                   v * math.sin(a + math.pi/4) - w ]

        dirs = [s > 0 for s in speeds]
        pwms = [abs(s) for s in speeds]

        self.driveFK(pwms, dirs)

    def stop(self):
        for wheel in self._wheels:
            wheel.setPWM(0)
            wheel.setDir(0)

chassis = MecanumChassis(PWM_PINS, DIR_PINS)

def mecanum_node():
    rospy.loginfo("Starting mecanum node...")
    rospy.init_node('mecanum', anonymous=True)
    rospy.Subscriber('cmd_vel', Twist, chassis.driveIK)
    rospy.spin()

if __name__ == '__main__':
    try:
        wiringpi.wiringPiSetupGpio()
        mecanum_node()
    except rospy.ROSInterruptException:
        rospy.loginfo("Stopping mecanum node...")
    finally:
        chassis.stop()
