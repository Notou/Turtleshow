#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
import sys
import numpy as np
import math
from Tkinter import *
import ttk as tk
import threading
from subprocess import call, check_output
import re

class ui(Frame):
    def __init__(self, master):
        rospy.init_node('sound_remote', anonymous=True)
        self.pub = rospy.Publisher("/turtleshow/text_to_say", String, queue_size=10)
        self.pubSound = rospy.Publisher("/turtleshow/sound_to_play", String, queue_size=10)
        self.pubSwitchVideo = rospy.Publisher("/turtleshow/video_on", Bool, queue_size=10)
        self.pubSwitchMovement = rospy.Publisher("/turtleshow/movement_on", Bool, queue_size=10)

        rospy.Subscriber("/turtleshow/robot_charge_level", Point, self.BatteryChargeCallback)

        self.pattern = re.compile('[0-9]+%')

        self.soundsFolder = "sons"
        self.textButton01 = "bruit.wav"
        self.textButton02 = "sncf.wav"
        self.textButton03 = "vache.wav"
        self.textButton04 = "barbare-camarchepas.wav"
        self.textButton05 = "naindows-corbeille01.wav"
        self.textButton06 = "naindows-demarrage01.wav"
        self.textButton07 = "barbare-dring.wav"
        self.textButton08 = "naindows-findesession01.wav"
        self.textButton09 = "barbare-frappe01.wav"
        self.textButton10 = "barbare-pasdormir.wav"
        self.textButton11 = "barbare-tavutatete.wav"
        # self.textButton12 = "barbare-tavutatete.wav"


        Frame.__init__(self, master)
        self.config(height = 50, width = 100)
        self.pack(fill = BOTH, expand = True)
        self.createWidgets()

    def createWidgets(self):
        self.topFrame = tk.Frame(self)
        self.topFrame.pack(fill = BOTH, expand = True)
        #Sounds Buttons
        self.soundButtonsFrame = tk.Frame(self.topFrame)
        self.soundButtonsFrame.pack(side = LEFT, fill = X, expand = True)
        self.button01 = tk.Button(self.soundButtonsFrame, command = self.callbackButton01, text = self.textButton01)
        self.button01.grid()
        self.button02 = tk.Button(self.soundButtonsFrame, command = self.callbackButton02, text = self.textButton02)
        self.button02.grid()
        self.button03 = tk.Button(self.soundButtonsFrame, command = self.callbackButton03, text = self.textButton03)
        self.button03.grid()
        self.button04 = tk.Button(self.soundButtonsFrame, command = self.callbackButton04, text = self.textButton04)
        self.button04.grid()
        self.button05 = tk.Button(self.soundButtonsFrame, command = self.callbackButton05, text = self.textButton05)
        self.button05.grid()
        self.button06 = tk.Button(self.soundButtonsFrame, command = self.callbackButton06, text = self.textButton06)
        self.button06.grid()
        self.button07 = tk.Button(self.soundButtonsFrame, command = self.callbackButton07, text = self.textButton07)
        self.button07.grid()
        self.button08 = tk.Button(self.soundButtonsFrame, command = self.callbackButton08, text = self.textButton08)
        self.button08.grid()
        self.button09 = tk.Button(self.soundButtonsFrame, command = self.callbackButton09, text = self.textButton09)
        self.button09.grid()
        self.button10 = tk.Button(self.soundButtonsFrame, command = self.callbackButton10, text = self.textButton10)
        self.button10.grid()
        self.button11 = tk.Button(self.soundButtonsFrame, command = self.callbackButton11, text = self.textButton11)
        self.button11.grid()
        # self.button12 = tk.Button(command = self.callbackButton12, text = self.textButton12)
        # self.button12.grid()

        #Affichage de la charge
        self.stateDisplayFrame = tk.Frame(self.topFrame)
        self.stateDisplayFrame.pack(side = LEFT, fill = X, expand = True)
        self.turtlebotChargeLabel = tk.Label(self.stateDisplayFrame, text = "Charge du turtlebot: " + "---")
        self.turtlebotChargeLabel.grid()
        self.sceneLaptopChargeLabel = tk.Label(self.stateDisplayFrame, text = "Charge de l'ordinateur sur scene: " + "---")
        self.sceneLaptopChargeLabel.grid()
        self.regieLaptopChargeLabel = tk.Label(self.stateDisplayFrame, text = "Charge de l'ordinateur de régie: " + "---")
        self.regieLaptopChargeLabel.grid()

        #Boutons de commande
        self.actionButtonsFrame = tk.Frame(self.topFrame)
        self.actionButtonsFrame.pack(side = RIGHT, fill = X, expand = True)
        self.videoSwitchButton = Button(self.actionButtonsFrame, command = self.VideoSwitchCallback, background = 'red', activebackground = 'red', text = "Turn video ON")
        self.videoSwitchButton.grid()
        self.videoOn = False
        self.gotToBaseButton = Button(self.actionButtonsFrame, command = self.GotToBaseCallback, text = "Aller à la base")
        self.gotToBaseButton.grid(pady = 50)
        self.movementSwitchButton = Button(self.actionButtonsFrame, command = self.MoveSwitchCallback, background = 'red', activebackground = 'red', text = "Turn movement ON")
        self.movementSwitchButton.grid()
        self.movementOn = False

        self.textEntry = Text(self, wrap=WORD, padx = 10, pady = 10)
        self.textEntry.pack(side = BOTTOM, fill = BOTH, expand = True)
        self.bind_all('<KeyPress-Return>', self._enterHandler)

    def _enterHandler(self,event):
        self.toSend = String()
        self.toSend.data = self.textEntry.get("1.0",'end-1c').replace('\n', ' ').replace('\r', '')
        if self.toSend.data != "":
            self.pub.publish(self.toSend)

    def callbackButton01(self):
        self.toSend = String()
        self.toSend.data = self.soundsFolder + "/" + self.textButton01
        self.pubSound.publish(self.toSend)

    def callbackButton02(self):
        self.toSend = String()
        self.toSend.data = self.soundsFolder + "/" + self.textButton02
        self.pubSound.publish(self.toSend)

    def callbackButton03(self):
        self.toSend = String()
        self.toSend.data = self.soundsFolder + "/" + self.textButton03
        self.pubSound.publish(self.toSend)

    def callbackButton04(self):
        self.toSend = String()
        self.toSend.data = self.soundsFolder + "/" + self.textButton04
        self.pubSound.publish(self.toSend)

    def callbackButton05(self):
        self.toSend = String()
        self.toSend.data = self.soundsFolder + "/" + self.textButton05
        self.pubSound.publish(self.toSend)

    def callbackButton06(self):
        self.toSend = String()
        self.toSend.data = self.soundsFolder + "/" + self.textButton06
        self.pubSound.publish(self.toSend)

    def callbackButton07(self):
        self.toSend = String()
        self.toSend.data = self.soundsFolder + "/" + self.textButton07
        self.pubSound.publish(self.toSend)

    def callbackButton08(self):
        self.toSend = String()
        self.toSend.data = self.soundsFolder + "/" + self.textButton08
        self.pubSound.publish(self.toSend)

    def callbackButton09(self):
        self.toSend = String()
        self.toSend.data = self.soundsFolder + "/" + self.textButton09
        self.pubSound.publish(self.toSend)

    def callbackButton10(self):
        self.toSend = String()
        self.toSend.data = self.soundsFolder + "/" + self.textButton10
        self.pubSound.publish(self.toSend)

    def callbackButton11(self):
        self.toSend = String()
        self.toSend.data = self.soundsFolder + "/" + self.textButton11
        self.pubSound.publish(self.toSend)

    # def callbackButton12(self):
    #     self.toSend = String()
    #     self.toSend.data = self.soundsFolder + "/" + self.textButton12
    #     self.pubSound.publish(self.toSend)

    def BatteryChargeCallback(self, msg):
        regieCharge = int(self.pattern.findall(check_output("acpi"))[0].rstrip('%'))
        self.regieLaptopChargeLabel.config(text = "Charge de l'ordinateur de régie: " + str(regieCharge) + "%")
        self.sceneLaptopChargeLabel.config(text = "Charge de l'ordinateur sur scene: " + str(msg.y) + "%")
        self.turtlebotChargeLabel.config(text = "Charge du turtlebot: " + str(msg.x) + "%")
        if regieCharge < 20:
            self.regieLaptopChargeLabel.config( foreground = 'red')
        elif regieCharge < 50:
            self.regieLaptopChargeLabel.config( foreground = 'orange')
        else:
            self.regieLaptopChargeLabel.config( foreground = 'green')

        if msg.y < 20:
            self.sceneLaptopChargeLabel.config( foreground = 'red')
        elif msg.y < 50:
            self.sceneLaptopChargeLabel.config( foreground = 'orange')
        else:
            self.sceneLaptopChargeLabel.config( foreground = 'green')

        if msg.x < 20:
            self.turtlebotChargeLabel.config( foreground = 'red')
        elif msg.x < 50:
            self.turtlebotChargeLabel.config( foreground = 'orange')
        else:
            self.turtlebotChargeLabel.config( foreground = 'green')

    def VideoSwitchCallback(self):
        toSend = Bool()
        if self.videoOn:
            self.videoSwitchButton.config( activebackground = 'red', background = 'red', text = "Turn video ON")
            self.videoOn = False
        else:
            self.videoSwitchButton.config( activebackground = 'green', background = 'green', text = "Turn video OFF")
            self.videoOn = True
        toSend.data = self.videoOn
        self.pubSwitchVideo.publish(toSend)

    def GotToBaseCallback(self):
        soundThread = threading.Thread(target=self.GoToBase)
        soundThread.daemon = True
        soundThread.start()

    def GoToBase(self):
        call(["roslaunch", "kobuki_auto_docking", "activate.launch", "--screen"])


    def MoveSwitchCallback(self):
        toSend = Bool()
        if self.movementOn:
            self.movementSwitchButton.config( activebackground = 'red', background = 'red', text = "Turn movement ON")
            self.movementOn = False
        else:
            self.movementSwitchButton.config( activebackground = 'green', background = 'green', text = "Turn movement OFF")
            self.movementOn = True
        toSend.data = self.movementOn
        self.pubSwitchMovement.publish(toSend)


if __name__ == '__main__':
    window = ui(None)
    window.master.title('Boite à paroles')
    window.mainloop()
