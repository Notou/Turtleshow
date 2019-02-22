#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
import rospy
from std_msgs.msg import String, Bool, Int8
from geometry_msgs.msg import Point
from subprocess import call
import threading
import vlc
import time
from optparse import OptionParser


class sound_player():
    def __init__(self):
        # Constants
        self.storagePath = "/raccourci_video_robot/"
        self.speakingVideoPath = "video_parle.mp4"
        self.idleVideoPath = "video_tait.mp4"
        self.tiredVideoPath = "video_fatigue.mp4"
        self.baillementSoundPath = "son_fatigue.wav"

        # Init variables
        self.laptopChargeLow = False
        self.turtlebotChargeLow = False
        self.isPlaying = False
        self.videoOn = False
        self.playbackVolume = 1

        # Parse arguments
        options, _ = argument_parser().parse_args()
        self.video = options.video
        self.playbackVolume = options.audio
        zoom = options.zoom
        self.id = options.id
        self.number = options.number

        rospy.init_node('sound_player', anonymous=True)
        rospy.Subscriber("/turtleshow/text_to_say", String, self.callback)
        rospy.Subscriber("/turtleshow/sound_to_play", String, self.callbackSound)
        if self.video:
            rospy.Subscriber("/turtleshow/video_on", Bool, self.callbackSwitchVideo)
        rospy.Subscriber("/turtleshow/robot_charge_level", Point, self.BatteryChargeCallback)
        rospy.Subscriber("/turtleshow/synchro", Int8, self.callbackSynchro)

        # Handle synchro
        self.syncMsg = Int8(self.id)
        self.pubSynchro = rospy.Publisher("/turtleshow/synchro", Int8, queue_size=10)
        self.syncArray = zeroes(self.number)


        if zoom == 1:
            self.interface = vlc.Instance('--no-audio', '--input-repeat=-1', '--no-video-title-show', '--fullscreen', '--mouse-hide-timeout=0')
            self.player=self.interface.media_player_new()
            self.player.toggle_fullscreen()
        else:
            self.interface = vlc.Instance('--no-audio', '--input-repeat=-1', '--video-title-show', '--mouse-hide-timeout=0', '--video-title= ', '--zoom', str(zoom))
            self.player=self.interface.media_player_new()

        self.mediaSpeaking = self.interface.media_new(self.storagePath+self.speakingVideoPath)
        self.mediaIdle = self.interface.media_new(self.storagePath+self.idleVideoPath)
        self.mediaTired = self.interface.media_new(self.storagePath+self.tiredVideoPath)
        self.player.set_media(self.mediaIdle)
        if self.videoOn:
            self.player.play()

        rospy.loginfo("Sound self.player launched")
        rospy.spin()

    def callback(self, msg):
        print msg.data

        self.filePath = self.storagePath + "toBePlayed.wav"
        exitCode = call(["pico2wave", "-w", self.filePath, "-l", "fr-FR", msg.data])
        if exitCode != 0:
            rospy.logerr("Erreur à la création du fichier son!")

        time.sleep(1)
        self.syncArray = zeroes(self.number)
        self.pubSynchro.publish(self.syncMsg)


    def callbackSound(self, msg):
        print msg.data

        self.filePath = self.storagePath + msg.data

        soundThread = threading.Thread(target=self.launchButtonSound)
        soundThread.daemon = True
        soundThread.start()

    def callbackSwitchVideo(self, msg):
        self.videoOn = msg.data
        if not self.videoOn:
            self.player.stop()
        if self.videoOn:
            self.player.play()

    def callbackSynchro(self, msg):
        self.syncArray[msg.data] = 1
        if fullArray(self.syncArray):
            soundThread = threading.Thread(target=self.launchSound)
            soundThread.daemon = True
            soundThread.start()

    def launchButtonSound(self):
        print self.isPlaying
        if self.isPlaying:
            rospy.logwarn("Il y a déjà un son en train d'être joué")
            return

        self.isPlaying = True

        exitCode = call(["play", "-D", self.filePath, "speed", "0.9", "vol", str(self.playbackVolume)])
        if exitCode != 0:
            rospy.logerr("Erreur à l'émission du son!")
        self.isPlaying = False

    def launchSound(self):
        print self.isPlaying
        if self.isPlaying:
            rospy.logwarn("Il y a déjà un son en train d'être joué")
            return

        self.isPlaying = True

        self.player.set_media(self.mediaSpeaking)
        if self.videoOn:
            self.player.play()
        exitCode = call(["play", "-D", self.filePath, "reverse", "trim", "0.4","reverse", "speed", "0.9", "vol", str(self.playbackVolume)]) #Permet de couper avant la fin du fichier (ici 0.4s)
        if exitCode != 0:
            rospy.logerr("Erreur à l'émission du son!")
        self.player.set_media(self.mediaIdle)
        if self.videoOn:
            self.player.play()
        self.isPlaying = False

    def launchBaillementSound(self):

        self.player.set_media(self.mediaTired)
        self.player.play()
        exitCode = call(["play", "-D", self.filePath, "speed", "0.9", "vol", str(self.playbackVolume)])
        if exitCode != 0:
            rospy.logerr("Erreur à l'émission du son!")
        self.player.set_media(self.mediaIdle)
        if self.videoOn:
            self.player.play()
        else:
            self.player.stop()
        self.isPlaying = False

    def BatteryChargeCallback(self, msg):
        if msg.y < 20:
            if not self.laptopChargeLow:
                if not self.isPlaying:
                    self.isPlaying = True
                    self.laptopChargeLow = True
                    self.filePath = self.storagePath + self.baillementSoundPath
                    soundThread = threading.Thread(target=self.launchBaillementSound)
                    soundThread.daemon = True
                    soundThread.start()
        else:
            if self.laptopChargeLow:
                self.laptopChargeLow = False

        if msg.x < 20:
            if not self.turtlebotChargeLow:
                if not self.isPlaying:
                    self.isPlaying = True
                    self.turtlebotChargeLow = True
                    self.filePath = self.storagePath + self.baillementSoundPath
                    soundThread = threading.Thread(target=self.launchBaillementSound)
                    soundThread.daemon = True
                    soundThread.start()
        else:
            if self.turtlebotChargeLow:
                self.turtlebotChargeLow = False

def fullArray(array):
    for i in array:
        if i < 1:
            return False
    return True

def zeroes(nb):
    array = []
    for i in range(nb):
        array.append(0)
    return array

def argument_parser():
    parser = OptionParser(usage="%prog: [options]")
    parser.add_option(
        "-v", "--no-video", dest="video", action="store_false", default=True,
        help="Turn off video output")
    parser.add_option(
        "-z", "--video-zoom", dest="zoom", type="float", default=1,
        help="Set video zoom [default=%default]")
    parser.add_option(
        "-a", "--audio-volume", dest="audio", type="float", default=1,
        help="Set audio volume [default=%default]")
    parser.add_option(
        "-n", "--nb-process", dest="number", type="int", default=3,
        help="number of sound self.player instances [default=%default]")
    parser.add_option(
        "-i", "--id-proc", dest="id", type="int", default=0,
        help="id of this instance [default=%default]")

    return parser

if __name__ == '__main__':
    listener = sound_player()
