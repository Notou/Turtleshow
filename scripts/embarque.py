#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import sys, tty, termios
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
#import cv2.cv as cv
import math


pub = rospy.Publisher("/cmd_vel_mux/input/teleop",Twist, queue_size=10)
maxSpeed = 0.25             # Vitesse maximale
mediumSpeed = 0.2           # Vitesse moyennement limitée
slowSpeed = 0.1             # Vitesse très limitée
emergencyStopDistance = 0.5 # Distance à laquelle le robot s'arrete net
StopDistance = 0.51         # Distance à laquelle le robot freine gentiment pour s'arreter
MediumDistance = 1          # Distance au delà de laquelle le robo va à mediumSpeed
FreedomDistance = 1.5       # Distance au delà de laquelle le robot va à maxSpeed
global currSpeed
currSpeed= 0
global currSpeedRad
currSpeedRad= 0
global lastDirection
lastDirection = "droite"
global chgDirCounter
chgDirCounter = 0
global rotation
rotation = False
global autonomousMode
autonomousMode = False
global joyTwist


def callback(msg):
    global currSpeed
    global pub
    global lastDirection
    global rotation
    global chgDirCounter
    global currSpeedRad
    global autonomousMode
    global joyTwist
    bridge = CvBridge()
    twist = Twist()
    # Use cv_bridge() to convert the ROS image to OpenCV format
    try:
        # The depth image is a single-channel float32 image
        depth_image = bridge.imgmsg_to_cv2(msg, "32FC1")
    except CvBridgeError, e:
        print e

    if autonomousMode == False:
        pub.publish(joyTwist)
        return

    # depth_image.height = height of the matrix
    # depth_image.width = width of the matrix
    # depth_image[x,y] = the float value in m of a point place a a height x and width y

    # Get min dist
    print np.asarray(depth_image).shape
    (minVal,maxVal,minLoc,maxLoc) = cv2.minMaxLoc(np.asarray(depth_image[0:250]))
    print "Loc : "+str(minLoc)+"Val : "+str(minVal)

    twist.linear.x = currSpeed
    twist.angular.z = currSpeedRad

    # Gestion demi tour
    if chgDirCounter <= 0:
        chgDirCounter = 0
    print "Counter : "+str(chgDirCounter)
    if rotation == True:
        accelerationRadiale(-0.6)
        acceleration(0)
        chgDirCounter = chgDirCounter - 0.5
        if chgDirCounter <= 0:
            rotation = False
        pub.publish(twist)
        return
    # Partie Rotation
    if maxVal < 1.6:
        print "demi tour"
        chgDirCounter = 50
        rotation = True
    elif chgDirCounter > 50:
        print "demi tour (counter)"
        rotation = True
    elif minLoc[0] < 320 and minVal < 1.5 :
        print "Je dois tourner a droite"
        if lastDirection == "gauche":
            chgDirCounter = chgDirCounter + 10
            lastDirection = "droite"
        accelerationRadiale(-0.5)
    elif minLoc[0] > 320 and minVal < 1.5 :
        print "Je dois tourner a gauche"
        if lastDirection == "droite":
            chgDirCounter = chgDirCounter + 10
            lastDirection = "gauche"
        accelerationRadiale(0.5)
    else:
        accelerationRadiale(0)

    # Partie translation
    if minVal < emergencyStopDistance:
        currSpeed = 0
        print "Emergency stop!!!"
    elif minVal < StopDistance:
        print "Stop"
        acceleration(0)
    elif minVal < MediumDistance:
        acceleration(slowSpeed)
        chgDirCounter = chgDirCounter - 2
    elif minVal < FreedomDistance:
        acceleration(mediumSpeed)
        chgDirCounter = chgDirCounter - 3
    else:
        acceleration(maxSpeed)
        chgDirCounter = chgDirCounter - 4


    pub.publish(twist)

def acceleration(targetSpeed):
    global currSpeed
    if targetSpeed>maxSpeed:
        targetSpeed = maxSpeed
    if targetSpeed< 0:
        targetSpeed = 0

    if currSpeed > targetSpeed:
        currSpeed = currSpeed - 0.02
    elif currSpeed < targetSpeed:
        currSpeed = currSpeed + 0.01

def accelerationRadiale(targetSpeed):
    global currSpeedRad

    if currSpeedRad > targetSpeed:
        currSpeedRad = currSpeedRad - 0.02
    elif currSpeedRad < targetSpeed:
        currSpeedRad = currSpeedRad + 0.02

def joystickCallback(msg):
    global autonomousMode
    global joyTwist
    joyTwist = msg
    if msg.linear.y == -1:
        autonomousMode = False
    if msg.linear.y == 1:
        autonomousMode = True

def listener():
    print "lanching nav node"
    rospy.init_node('nav', anonymous=True)
    rospy.Subscriber("/camera/depth/image", Image, callback)
    rospy.Subscriber("/turtleshow/command", Twist, joystickCallback)
    rospy.spin()

if __name__ == '__main__':
    print "main!"
    listener()
