#!/usr/bin/env python

import math
import webiopi
import rospy

from geometry_msgs.msg import Twist

def setup():
    rospy.init_node('webiopy')


def loop():
    webiopi.sleep(1)


def destroy():
    pass


@webiopi.macro
def servoUp(servo):
    return "oks"


@webiopi.macro
def servoDown(servo):
    return "oks"


@webiopi.macro
def servoStop(servo):
    return "oks"


@webiopi.macro
def servoPosition(servo, pos):
    return "oks"


@webiopi.macro
def manipulatorVelocity(vel):
    return "oks"


@webiopi.macro
def manipulatorOff(id):
    return "oks"


@webiopi.macro
def mecanumDriveIK(angle, velocity, rotation):
    p = rospy.Publisher('cmd_vel', Twist)
    twist = Twist()
    twist.linear.y = velocity * math.sin(angle)
    twist.linear.x = velocity * math.cos(angle)
    twist.angular.z = rotation
    p.publish(twist)
    return "oks"


@webiopi.macro
def notImplemented():
    return "not implemented"