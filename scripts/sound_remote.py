#!/usr/bin/env python3
# This Python file uses the following encoding: utf-8
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
import sys
import numpy as np
import math
from tkinter import *
import tkinter.ttk as tk
import threading
from subprocess import call, check_output
import re
import json
from datetime import datetime


class ui(Frame):
    def __init__(self, master):
        rospy.init_node('sound_remote', anonymous=True)
        self.pub = rospy.Publisher("/turtleshow/text_to_say", String, queue_size=10)
        self.pubSound = rospy.Publisher("/turtleshow/sound_to_play", String, queue_size=10)
        self.pubSwitchVideo = rospy.Publisher("/turtleshow/video_on", Bool, queue_size=10)
        self.pubSwitchMovement = rospy.Publisher("/turtleshow/movement_on", Bool, queue_size=10)

        rospy.Subscriber("/turtleshow/robot_charge_level", Point, self.BatteryChargeCallback)

        self.pattern = re.compile('[0-9]+%')
        self.storagePath = "/raccourci_video_robot/"
        self.soundsFolder = "sons"
        self.columnNumber = 10  #Nombre de colonnes de boutons

        # self.listOfButtons = list()
        #
        # self.listOfButtons.append({"Nom":"Bruits","Type":"Son","Scene":1,"Texte":"bruit.wav"})
        # self.listOfButtons.append({"Nom":"SNCF","Type":"Son","Scene":1,"Texte":"sncf.wav"})
        # self.listOfButtons.append({"Nom":"Carne","Type":"Son","Scene":1,"Texte":"vache.wav"})
        # self.listOfButtons.append({"Nom":"Texte","Type":"Texte","Scene":1,"Texte":"Ce texte à été envoyé depuis un bouton."})
        #
        #
        # # Open a file for writing
        # out_file = open(self.storagePath + "config.json","w")
        #
        # # Save the dictionary into this file
        # # (the 'indent=4' is optional, but makes it more readable)
        # json.dump(self.listOfButtons,out_file, indent=4, ensure_ascii=False)
        #
        # # Close the file
        # out_file.close()

        # Open the file for reading
        in_file = open(self.storagePath + "config.json","r")
        # Load the contents from the file, which creates a new dictionary
        self.listOfButtons = json.load(in_file)
        # Close the file... we don't need it anymore
        in_file.close()


        Frame.__init__(self, master)
        self.config(height = 50, width = 100)
        self.pack(fill = BOTH, expand = True)
        self.createWidgets()

    def createWidgets(self):
        self.topFrame = tk.Frame(self)
        self.topFrame.pack(fill = BOTH, expand = True)
        #Sounds Buttons
        self.tabs = tk.Notebook(self.topFrame)
        self.tabs.pack(side = TOP, fill = BOTH, expand = True)

        self.listOfTabs = dict()
        for b in self.listOfButtons:
            if not b["Scene"] in self.listOfTabs:
                self.listOfTabs[b["Scene"]] = dict()
                self.listOfTabs[b["Scene"]]["tab"] = tk.Frame(self.tabs)
                self.listOfTabs[b["Scene"]]["ButtonNumber"]= 0
            button = tk.Button(self.listOfTabs[b["Scene"]]["tab"], command = (lambda type=b["Type"], text=b["Texte"]: self.callbackButton(type,text)), text = b["Nom"])
            button.grid(row = self.listOfTabs[b["Scene"]]["ButtonNumber"] // self.columnNumber,column = self.listOfTabs[b["Scene"]]["ButtonNumber"] % self.columnNumber, padx = 5, pady = 5)
            self.listOfTabs[b["Scene"]]["ButtonNumber"] = self.listOfTabs[b["Scene"]]["ButtonNumber"] + 1;
        for tab in sorted(self.listOfTabs.keys()):
            self.tabs.add(self.listOfTabs[tab]["tab"], text="Q" + str(tab))


        #Affichage de la charge
        self.stateDisplayFrame = tk.Frame(self.topFrame)
        self.stateDisplayFrame.pack(side = LEFT, fill = X, expand = True)
        self.turtlebotChargeLabel = tk.Label(self.stateDisplayFrame, text = "Charge du turtlebot: " + "---")
        self.turtlebotChargeLabel.pack()
        self.sceneLaptopChargeLabel = tk.Label(self.stateDisplayFrame, text = "Charge de l'ordinateur sur scene: " + "---")
        self.sceneLaptopChargeLabel.pack()
        self.regieLaptopChargeLabel = tk.Label(self.stateDisplayFrame, text = "Charge de l'ordinateur de régie: " + "---")
        self.regieLaptopChargeLabel.pack()

        #Boutons de commande
        self.actionButtonsFrame = tk.Frame(self.topFrame)
        self.actionButtonsFrame.pack(side = RIGHT, fill = BOTH, expand = True)
        self.videoSwitchButton = Button(self.actionButtonsFrame, command = self.VideoSwitchCallback, background = 'red', activebackground = 'red', text = "Turn video ON")
        self.videoSwitchButton.pack()
        self.videoOn = False
        self.gotToBaseButton = Button(self.actionButtonsFrame, command = self.GotToBaseCallback, text = "Aller à la base")
        self.gotToBaseButton.pack(pady = 10)
        self.movementSwitchButton = Button(self.actionButtonsFrame, command = self.MoveSwitchCallback, background = 'red', activebackground = 'red', text = "Turn movement ON")
        self.movementSwitchButton.pack()
        self.movementOn = False

        self.textEntry = Text(self, wrap=WORD, padx = 10, pady = 10)
        self.textEntry.pack(side = BOTTOM, fill = BOTH, expand = True, ipady = 20)
        self.bind_all('<KeyPress-Return>', self._enterHandler)

    def _enterHandler(self,event):
        self.toSend = String()
        self.toSend.data = self.textEntry.get("1.0",'end-1c').replace('\n', ' ').replace('\r', '')
        self.textEntry.delete("1.0",'end-1c')
        if self.toSend.data != "":
            self.pub.publish(self.toSend)
        with open(self.storagePath + 'historique_des_textes_entres.txt', 'a') as f:
            f.write(datetime.now().strftime('%Y-%m-%d %H:%M:%S') + " : " + self.toSend.data.encode('utf-8') + "\n")

    def callbackButton(self,type,text):
        self.toSend = String()
        if type == "Son":
            self.toSend.data = self.soundsFolder + "/" + text
            self.pubSound.publish(self.toSend)
        elif type == "Texte":
            self.pub.publish(text)


    def BatteryChargeCallback(self, msg):
        regieCharge = int(self.pattern.findall(check_output("acpi",text=True))[-1].rstrip('%'))
        self.regieLaptopChargeLabel.config(text = "Charge de l'ordinateur de régie: " + str(regieCharge) + "%")
        self.sceneLaptopChargeLabel.config(text = "Charge de l'ordinateur sur scene: " + str(msg.y) + "%")
        self.turtlebotChargeLabel.config(text = "Charge du turtlebot: " + str(msg.x) + "%")
        if regieCharge < 20:
            self.regieLaptopChargeLabel.config( foreground = 'red')
        elif regieCharge < 50:
            self.regieLaptopChargeLabel.config( foreground = 'brown')
        else:
            self.regieLaptopChargeLabel.config( foreground = 'green')

        if msg.y < 20:
            self.sceneLaptopChargeLabel.config( foreground = 'red')
        elif msg.y < 50:
            self.sceneLaptopChargeLabel.config( foreground = 'brown')
        else:
            self.sceneLaptopChargeLabel.config( foreground = 'green')

        if msg.x < 20:
            self.turtlebotChargeLabel.config( foreground = 'red')
        elif msg.x < 50:
            self.turtlebotChargeLabel.config( foreground = 'brown')
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
