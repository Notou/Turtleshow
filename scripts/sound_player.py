#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
import sys
import numpy as np
import math
from subprocess import call
import threading
import vlc


global isPlaying
isPlaying = False
global videoOn
videoOn = False
global shouldPlay
global storagePath
storagePath = "/raccourci_video_robot/"
global speakingVideoPath
speakingVideoPath = "video_parle.mp4"
global idleVideoPath
idleVideoPath = "video_tait.mp4"
global tiredVideoPath
tiredVideoPath = "video_fatigue.mp4"
baillementSoundPath = "son_fatigue.wav"
global filePath
global interface
global player
global mediaSpeaking
global mediaIdle
global mediaTired

global laptopChargeLow
laptopChargeLow = False
global turtlebotChargeLow
turtlebotChargeLow = False

def callback(msg):
    global storagePath
    global filePath
    print msg.data

    filePath = storagePath + "toBePlayed.wav"
    exitCode = call(["pico2wave", "-w", filePath, "-l", "fr-FR", msg.data])
    if exitCode != 0:
        rospy.logerr("Erreur à la création du fichier son!")

    soundThread = threading.Thread(target=launchSound)
    soundThread.daemon = True
    soundThread.start()

def callbackSound(msg):
    global storagePath
    global filePath
    print msg.data

    filePath = storagePath + msg.data

    soundThread = threading.Thread(target=launchButtonSound)
    soundThread.daemon = True
    soundThread.start()

def callbackSwitchVideo(msg):
    global player
    global videoOn
    videoOn = msg.data
    if not videoOn:
        player.stop()
    if videoOn:
        player.play()

def launchButtonSound():
    global isPlaying
    global filePath
    print isPlaying
    if isPlaying:
        rospy.logwarn("Il y a déjà un son en train d'être joué")
        return

    isPlaying = True

    exitCode = call(["play", "-D", filePath, "speed", "0.9"])
    if exitCode != 0:
        rospy.logerr("Erreur à l'émission du son!")
    isPlaying = False

def launchSound():
    global isPlaying
    global filePath
    global player
    global mediaTired
    global mediaIdle
    print isPlaying
    if isPlaying:
        rospy.logwarn("Il y a déjà un son en train d'être joué")
        return

    isPlaying = True

    player.set_media(mediaSpeaking)
    if videoOn:
        player.play()
    exitCode = call(["play", "-D", filePath, "reverse", "trim", "0.7","reverse", "speed", "0.9"])
    if exitCode != 0:
        rospy.logerr("Erreur à l'émission du son!")
    player.set_media(mediaIdle)
    if videoOn:
        player.play()
    isPlaying = False

def launchBaillementSound():
    global filePath
    global player
    global mediaTired
    global mediaIdle
    global isPlaying
    
    player.set_media(mediaTired)
    player.play()
    exitCode = call(["play", "-D", filePath, "speed", "0.9"])
    if exitCode != 0:
        rospy.logerr("Erreur à l'émission du son!")
    player.set_media(mediaIdle)
    if videoOn:
        player.play()
    else:
        player.stop()
    isPlaying = False

def BatteryChargeCallback(msg):
    global laptopChargeLow
    global turtlebotChargeLow
    global filePath
    global isPlaying
    if msg.y < 20:
        if not laptopChargeLow:
            if not isPlaying:
                isPlaying = True
                laptopChargeLow = True
                filePath = storagePath + baillementSoundPath
                soundThread = threading.Thread(target=launchBaillementSound)
                soundThread.daemon = True
                soundThread.start()
    else:
        if laptopChargeLow:
            laptopChargeLow = False
    
    if msg.x < 20:
        if not turtlebotChargeLow:
            if not isPlaying:
                isPlaying = True
                turtlebotChargeLow = True
                filePath = storagePath + baillementSoundPath
                soundThread = threading.Thread(target=launchBaillementSound)
                soundThread.daemon = True
                soundThread.start()
    else:
        if turtlebotChargeLow:
            turtlebotChargeLow = False


def listener():
    global storagePath
    global idleVideoPath
    global speakingVideoPath
    global interface
    global player
    global mediaSpeaking
    global mediaIdle
    global mediaTired
    rospy.init_node('sound_player', anonymous=True)
    rospy.Subscriber("/turtleshow/text_to_say", String, callback)
    rospy.Subscriber("/turtleshow/sound_to_play", String, callbackSound)
    rospy.Subscriber("/turtleshow/video_on", Bool, callbackSwitchVideo)
    rospy.Subscriber("/turtleshow/robot_charge_level", Point, BatteryChargeCallback)
    interface = vlc.Instance('--no-audio', '--input-repeat=-1', '--no-video-title-show', '--fullscreen', '--mouse-hide-timeout=0')
    player=interface.media_player_new()
    player.toggle_fullscreen()
    mediaSpeaking = interface.media_new(storagePath+speakingVideoPath)
    mediaIdle = interface.media_new(storagePath+idleVideoPath)
    mediaTired = interface.media_new(storagePath+tiredVideoPath)
    player.set_media(mediaIdle)
    if videoOn:
        player.play()

    rospy.loginfo("Sound player launched")
    rospy.spin()

if __name__ == '__main__':
    listener()
