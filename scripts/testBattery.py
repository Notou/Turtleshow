#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
import rospy
from geometry_msgs.msg import Point
from diagnostic_msgs.msg import DiagnosticArray
import sys, tty, termios
from subprocess import call, check_output
import re

global p
pubCharge = rospy.Publisher("/turtleshow/robot_charge_level", Point, queue_size=10)

def callback(msg):
    global p
    if msg.status[0].name == 'mobile_base_nodelet_manager: Battery':
        point = Point()
        point.x =  float(msg.status[0].values[1].value)
        point.y = int(p.findall(check_output("acpi"))[0].rstrip('%'))
        pubCharge.publish(point)


def listener():
    global p
    rospy.loginfo("lanching monitor node")
    rospy.init_node('batteryMonitor', anonymous=True)
    rospy.Subscriber("/diagnostics", DiagnosticArray, callback)
    p = re.compile('[0-9]+%')
    rospy.logwarn(str(int(p.findall(check_output("acpi"))[0].rstrip('%'))))
    point = Point()
    point.y = int(p.findall(check_output("acpi"))[0].rstrip('%'))
    pubCharge.publish(point)
    rospy.spin()

if __name__ == '__main__':
    listener()
