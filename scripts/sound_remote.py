#!/usr/bin/env python3
# This Python file uses the following encoding: utf-8
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
import sys
import numpy as np
import math
from tkinter import *
import tkinter as tk
import tkinter.ttk as ttk
import threading
from subprocess import call, check_output
import re
import json
from datetime import datetime


class Gui():
    def __init__(self, root):
        rospy.init_node('sound_remote', anonymous=True)
        self.pub = rospy.Publisher("/turtleshow/text_to_say", String, queue_size=10)
        self.pubSound = rospy.Publisher("/turtleshow/sound_to_play", String, queue_size=10)
        self.pubSwitchVideo = rospy.Publisher("/turtleshow/video_on", Bool, queue_size=10)
        self.pubSwitchMovement = rospy.Publisher("/turtleshow/movement_on", Bool, queue_size=10)

        self.pattern = re.compile('[0-9]+%')
        self.storagePath = "/raccourci_video_robot/"
        self.soundsFolder = "sons"
        self.columnNumber = 5  #Nombre de colonnes de boutons
        self.charge_labels = {
            'Turtlebot': None,
            'Ordi scène': None,
            'Ordi régie': None,
        }

        self.build_gui(root)

        rospy.Subscriber('/turtleshow/robot_charge_level', Point, self.battery_callback)

    def build_gui(self, root):
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)

        mainframe = ttk.Frame(root)
        mainframe.grid(column=0, row=0, sticky='nwes')
        for i in range(self.columnNumber):
            mainframe.columnconfigure(i, weight=1, minsize=100)

        # Sounds Tabs & Buttons
        self.tabs = ttk.Notebook(mainframe)
        self.tabs.grid(column=0, row=0, columnspan=self.columnNumber, sticky='nwes')

        #Affichage de la charge
        charge_frame = ttk.LabelFrame(mainframe, text='Battery Charge', padding=5)
        charge_frame.grid(column=0, row=1, columnspan=3, sticky='nwes')

        for col, name in enumerate(self.charge_labels):
            charge_frame.columnconfigure(col, weight=1)
            label = ttk.Label(charge_frame, text=name)
            charge = ttk.Label(charge_frame, text='')
            label.grid(column=col, row=0, sticky='nesw')
            charge.grid(column=col, row=1, sticky='nesw')
            self.charge_labels[name] = charge

        #Boutons de commande
        self.actionButtonsFrame = ttk.Frame(mainframe)
        self.actionButtonsFrame.pack(side = RIGHT, fill = BOTH, expand = True)
        self.videoSwitchButton = Button(self.actionButtonsFrame, command = self.VideoSwitchCallback, background = 'red', activebackground = 'red', text = "Turn video ON")
        self.videoSwitchButton.pack()
        self.videoOn = False
        self.gotToBaseButton = Button(self.actionButtonsFrame, command = self.GotToBaseCallback, text = "Aller à la base")
        self.gotToBaseButton.pack(pady = 10)
        self.movementSwitchButton = Button(self.actionButtonsFrame, command = self.MoveSwitchCallback, background = 'red', activebackground = 'red', text = "Turn movement ON")
        self.movementSwitchButton.pack()
        self.movementOn = False

        self.textEntry = tk.Text(mainframe, wrap=tk.WORD, padx=10, pady=10)
        self.textEntry.pack(side = BOTTOM, fill = BOTH, expand = True, ipady = 20)

        self.draw_tabs()
        root.bind_all('<KeyPress-Return>', self._enterHandler)

    def draw_tabs(self):
        tabs = self.tabs.winfo_children()

        with open(self.storagePath + 'config.json') as config:
            config_dict = json.load(config)

        for name, buttons in config_dict.items():
            tab = ttk.Frame(self.tabs)
            self.tabs.add(tab, text=name)
            for i, btn in enumerate(buttons):
                button = ttk.Button(tab, text=btn['Nom'], command=(lambda _type=btn['Type'], text=btn['Texte']: self.callbackButton(_type,text)))
                r, c = divmod(i, self.columnNumber)
                button.grid(row=r, column=c, padx=5, pady=5)

    def _enterHandler(self,event):
        self.toSend = String()
        self.toSend.data = self.textEntry.get("1.0",'end-1c').replace('\n', ' ').replace('\r', '')
        self.textEntry.delete("1.0",'end-1c')
        if self.toSend.data != "":
            self.pub.publish(self.toSend)
        with open(self.storagePath + 'historique_des_textes_entres.txt', 'a') as f:
            pass #f.write(datetime.now().strftime('%Y-%m-%d %H:%M:%S') + " : " + self.toSend.data.encode('utf-8') + "\n")

    def callbackButton(self,type,text):
        self.toSend = String()
        if type == "Son":
            self.toSend.data = self.soundsFolder + "/" + text
            self.pubSound.publish(self.toSend)
        elif type == "Texte":
            self.pub.publish(text)


    def battery_callback(self, msg):
        charge_regie = int(self.pattern.findall(check_output("acpi",text=True))[-1].rstrip('%'))

        for label, val in zip(self.charge_labels.values(), [msg.x, msg.y, charge_regie]):
            val = round(val)
            color = 'red' if val < 20 else 'black' if val < 70 else 'green'
            label.config(text=f'{val} %', foreground=color)

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
    root = tk.Tk()
    root.title('Boite à paroles')
    Gui(root)
    root.mainloop()
