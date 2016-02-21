#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist

GPIO = webiopi.GPIO


PWM1 = 7
PWM2 = 8
PWM3 = 25
PWM4 = 11

DIR1 = 27
DIR2 = 22
DIR3 = 23
DIR4 = 24

PWM_PINS = [PWM1, PWM2, PWM3, PWM4]
DIR_PINS = [DIR1, DIR2, DIR3, DIR4]


class Wheel:
    def __init__(self, pwm_pin, dir_pin):
        self._pwm_pin = pwm_pin
        self._dir_pin = dir_pin
        GPIO.setFunction(self._dir_pin, GPIO.OUT)
        GPIO.setFunction(self._pwm_pin, GPIO.PWM)
        GPIO.pulseRatio(self._pwm_pin, 0)

    def setPWM(self, pwm)
        GPIO.pulseRatio(self._pwm_pin, pwm)

    def setDir(self, direction)
        GPIO.digitalWrite(self._dir_pin, direction)


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
                   v * math.cos(a + math.pi/4) + w,
                   v * math.cos(a + math.pi/4) - w,
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
        mecanum_node()
    except rospy.ROSInterruptException:
        rospy.loginfo("Stopping mecanum node...")
    finally:
        chassis.stop()
