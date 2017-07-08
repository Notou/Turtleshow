#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
import rospy
from std_msgs.msg import String
import sys
import numpy as np
import math
from subprocess import call
import threading
import vlc


global isPlaying
isPlaying = False
global storagePath
storagePath = "/home/cyrille/Musique/Turtleshow/"
global speakingVideoPath
speakingVideoPath = "GeysirCropped.mp4"
global idleVideoPath
idleVideoPath = "PuffinConverted.mp4"
global filePath
global interface
global player
global mediaSpeaking
global mediaIdle

def callback(msg):
    global storagePath
    global filePath
    print "Message recieved"
    print msg.data

    filePath = storagePath + "toBePlayed.wav"
    exitCode = call(["pico2wave", "-w", filePath, "-l", "fr-FR", msg.data])
    if exitCode != 0:
        print "Erreur à la création du fichier son!"

    soundThread = threading.Thread(target=launchSound)
    soundThread.daemon = True
    soundThread.start()

def callbackSound(msg):
    global storagePath
    global filePath
    print "Message recieved"
    print msg.data

    filePath = storagePath + msg.data

    soundThread = threading.Thread(target=launchSound)
    soundThread.daemon = True
    soundThread.start()

def launchSound():
    global isPlaying
    global filePath
    global player
    global mediaSpeaking
    global mediaIdle
    print isPlaying
    if isPlaying:
        print "Il y a déjà un son en train d'être joué"
        return

    isPlaying = True
    player.set_media(mediaSpeaking)
    player.play()
    exitCode = call(["play", "-D", filePath, "speed", "0.9"])
    if exitCode != 0:
        print "Erreur à l'émission du son!"
    player.set_media(mediaIdle)
    player.play()
    isPlaying = False


def listener():
    global storagePath
    global idleVideoPath
    global speakingVideoPath
    global interface
    global player
    global mediaSpeaking
    global mediaIdle
    print "launching!"
    rospy.init_node('sound_player', anonymous=True)
    rospy.Subscriber("/turtleshow/text_to_say", String, callback)
    rospy.Subscriber("/turtleshow/sound_to_play", String, callbackSound)
    interface = vlc.Instance('--no-audio', '--input-repeat=-1', '--no-video-title-show', '--fullscreen', '--mouse-hide-timeout=0')
    player=interface.media_player_new()
    player.toggle_fullscreen()
    mediaSpeaking = interface.media_new(storagePath+speakingVideoPath)
    mediaIdle = interface.media_new(storagePath+idleVideoPath)
    player.set_media(mediaIdle)
    player.play()

    print "launched"
    rospy.spin()

if __name__ == '__main__':
    print "main!"
    listener()
