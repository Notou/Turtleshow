#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
import rospy
from std_msgs.msg import String
import sys
import numpy as np
import math
from subprocess import call
import ttk as tk
import threading

class ui(tk.Frame):
    def __init__(self, master):
        rospy.init_node('sound_remote', anonymous=True)
        self.pub = rospy.Publisher("/turtleshow/text_to_say", String, queue_size=10)
        self.pubSound = rospy.Publisher("/turtleshow/sound_to_play", String, queue_size=10)
        self.soundsFolder = "barbareSons"
        self.textButton01 = "naindows-chiantos.wav"
        self.textButton02 = "naindows-aie01.wav"
        self.textButton03 = "barbare-baston01.wav"
        self.textButton04 = "barbare-camarchepas.wav"
        self.textButton05 = "naindows-corbeille01.wav"
        self.textButton06 = "naindows-demarrage01.wav"
        self.textButton07 = "barbare-dring.wav"
        self.textButton08 = "naindows-findesession01.wav"
        self.textButton09 = "barbare-frappe01.wav"
        self.textButton10 = "barbare-pasdormir.wav"
        self.textButton11 = "barbare-tavutatete.wav"
        tk.Frame.__init__(self, master)
        self.config(height = 50, width = 200)
        self.grid()
        self.createWidgets()

    def createWidgets(self):
        self.button01 = tk.Button(command = self.callbackButton01, text = self.textButton01)
        self.button01.grid()
        self.button02 = tk.Button(command = self.callbackButton02, text = self.textButton02)
        self.button02.grid()
        self.button03 = tk.Button(command = self.callbackButton03, text = self.textButton03)
        self.button03.grid()
        self.button04 = tk.Button(command = self.callbackButton04, text = self.textButton04)
        self.button04.grid()
        self.button05 = tk.Button(command = self.callbackButton05, text = self.textButton05)
        self.button05.grid()
        self.button06 = tk.Button(command = self.callbackButton06, text = self.textButton06)
        self.button06.grid()
        self.button07 = tk.Button(command = self.callbackButton07, text = self.textButton07)
        self.button07.grid()
        self.button08 = tk.Button(command = self.callbackButton08, text = self.textButton08)
        self.button08.grid()
        self.button09 = tk.Button(command = self.callbackButton09, text = self.textButton09)
        self.button09.grid()
        self.button10 = tk.Button(command = self.callbackButton10, text = self.textButton10)
        self.button10.grid()
        self.button11 = tk.Button(command = self.callbackButton11, text = self.textButton11)
        self.button11.grid()
        self.textEntry = tk.Entry(width = 200)
        self.textEntry.grid()
        self.bind_all('<KeyPress-Return>', self._enterHandler)

    def _enterHandler(self,event):
        self.toSend = String()
        self.toSend.data = self.textEntry.get()
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



if __name__ == '__main__':
    window = ui(None)
    window.master.title('Boite à paroles')
    window.mainloop()
