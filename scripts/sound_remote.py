#!/usr/bin/env python3
# This Python file uses the following encoding: utf-8

import re
import json
import threading
from subprocess import call, check_output
from datetime import datetime

import tkinter as tk
from tkinter import font
import tkinter.ttk as ttk

import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point


class Gui():
    def __init__(self, root):
        rospy.init_node('sound_remote', anonymous=True)
        self.pub = rospy.Publisher('/turtleshow/text_to_say', String, queue_size=10)
        self.speech_pub = rospy.Publisher('/turtleshow/sound_to_play', String, queue_size=10)
        self.face_pub = rospy.Publisher('/turtleshow/video_on', Bool, queue_size=10)
        self.movement_pub = rospy.Publisher('/turtleshow/movement_on', Bool, queue_size=10)

        self.pattern = re.compile('[0-9]+%')
        self.storage_path = '/raccourci_video_robot/'
        self.sounds_folder = 'sons'
        self.column_count = 5  #Nombre de colonnes de boutons
        self.charge_labels = {
            'Turtlebot': None,
            'Ordi scène': None,
            'Ordi régie': None,
        }
        self.active_button = None
        self.movement_on = tk.IntVar()
        self.face_on = tk.IntVar()
        self.joystick_text = tk.StringVar()

        self.build_gui(root)

        rospy.Subscriber('/turtleshow/robot_charge_level', Point, self.battery_callback)
        rospy.Subscriber('/turtleshow/gui_command', Point, self.handle_joystick)
        rospy.Subscriber('/turtleshow/joystick_connected', Bool, self.joystick_callback)

    def build_gui(self, root):
        s = ttk.Style()
        s.map('TButton', background=[('selected', '#ffa0a0')])
        s.map('TCheckbutton', background=[('!selected', 'red'), ('selected', 'green')])

        label_font = (font.Font(font=s.lookup('TLabel', 'font'))).actual()
        label_font['weight'] = 'bold'
        label_font['size'] = 16
        title_font = font.Font(**label_font)
        label_font['size'] = 28
        value_font = font.Font(**label_font)
        s.configure('TCheckbutton', font=title_font, anchor=tk.CENTER, indicatormargin=5, padding=5)
        s.configure('ChargeLabel.TLabel', font=title_font, anchor=tk.CENTER)
        s.configure('ChargeValue.TLabel', font=value_font, anchor=tk.CENTER)

        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)

        mainframe = ttk.Frame(root)
        mainframe.grid(column=0, row=0, sticky='nwes')
        for i in range(self.column_count):
            mainframe.columnconfigure(i, weight=1, minsize=100)

        # Sounds Tabs & Buttons
        self.tabs = ttk.Notebook(mainframe)
        self.tabs.grid(column=0, row=0, columnspan=self.column_count, sticky='nwes')

        #Affichage de la charge
        charge_frame = ttk.LabelFrame(mainframe, text='Battery Charge', padding=5)
        charge_frame.grid(column=0, row=1, columnspan=3, sticky='nwes')

        for col, name in enumerate(self.charge_labels):
            charge_frame.columnconfigure(col, weight=1)
            label = ttk.Label(charge_frame, text=name, style='ChargeLabel.TLabel')
            charge = ttk.Label(charge_frame, text='', style='ChargeValue.TLabel')
            label.grid(column=col, row=0, sticky='nesw')
            charge.grid(column=col, row=1, sticky='nesw')
            self.charge_labels[name] = charge

        #Boutons de commande
        buttons_frame = ttk.Frame(mainframe, padding=5)
        buttons_frame.grid(column=3, row=1, columnspan=2, rowspan=2, sticky='nwes')
        for r in range(3):
            buttons_frame.columnconfigure(r, weight=1)
            buttons_frame.rowconfigure(r, weight=1)

        ttk.Button(buttons_frame, command=self.go_to_base, text="Aller à la base").grid(column=0, row=0, sticky='nwes')
        ttk.Button(buttons_frame, command=self.draw_tabs, text="Reload buttons").grid(column=1, row=0, sticky='nwes')
        self.face_button = ttk.Checkbutton(buttons_frame, command=self.toggle_face, variable=self.face_on, text='\u25CB Visage')
        self.movement_button = ttk.Checkbutton(buttons_frame, command=self.toggle_movement, variable=self.movement_on, text='X Move')

        self.face_button.grid(column=0, row=1, columnspan=2, sticky='nwes')
        self.movement_button.grid(column=0, row=2, columnspan=2, sticky='nwes')

        joy_text = ttk.Label(mainframe, textvariable=self.joystick_text)
        joy_text.grid(column=0, row=2, columnspan=3)
        self.entry = tk.Text(mainframe, wrap=tk.WORD, padx=10, pady=10)
        self.entry.grid(column=0, row=3, columnspan=self.column_count, sticky='nwes')

        self.draw_tabs()
        root.bind_all('<KeyPress-Return>', self.return_pressed)
        root.bind_all('<<NotebookTabChanged>>', self.tab_changed)

    def draw_tabs(self):
        tabs = self.tabs.winfo_children()
        self.active_button = None
        for tab in tabs:
            self.tabs.forget(tab)
            tab.destroy()

        with open(self.storage_path + 'config.json') as config:
            config_dict = json.load(config)

        for name, buttons in config_dict.items():
            tab = ttk.Frame(self.tabs)
            self.tabs.add(tab, text=name)
            for i, btn in enumerate(buttons):
                button = ttk.Button(tab, text=btn['Nom'], command=(lambda _type=btn['Type'], text=btn['Texte']: self.speak(_type,text)))
                r, c = divmod(i, self.column_count)
                button.grid(row=r, column=c, padx=5, pady=5)

        self.activate_button(self.tabs.winfo_children()[0].winfo_children()[0])

    def return_pressed(self, event):
        self.toSend = String()
        self.toSend.data = self.entry.get("1.0",'end-1c').replace('\n', ' ').replace('\r', '')
        self.entry.delete("1.0",'end-1c')
        if self.toSend.data != "":
            self.pub.publish(self.toSend)
        with open(self.storage_path + 'historique_des_textes_entres.txt', 'a') as f:
            pass #f.write(datetime.now().strftime('%Y-%m-%d %H:%M:%S') + " : " + self.toSend.data.encode('utf-8') + "\n")

    def handle_joystick(self, msg):
        value = int(msg.x)
        kind = int(msg.y)
        number = int(msg.z)
        
        if not value:
            return
        
        if kind & 0x01:    #button
            if number == 0:
                result = 'croix'
                self.movement_button.invoke()
            elif number == 1:
                result = 'rond'
                self.face_button.invoke()
            elif number == 2:
                result = 'triangle'
            elif number == 3:
                result = 'carré'
            elif number == 4:
                result = 'tl'
                if self.active_button:
                    self.active_button.invoke()
            elif number == 5:
                result = 'tr'
                if self.active_button:
                    self.active_button.invoke()
            else:
                result = f'unknown button number={number} value={value}'

        elif kind & 0x02:  #axis
            if number == 2 and value == 32767:
                result = 'gachette gauche'
                self.switch_button(-1)
            elif number == 5 and value == 32767:
                result = 'gachette droite'
                self.switch_button(1)
            elif number == 6:
                result = 'droite' if value > 0 else 'gauche'
                self.switch_tab() if value > 0 else self.switch_tab(forward=False)
            elif number == 7:
                result = 'bas' if value > 0 else 'haut'
            else:
                result = f'unknown axis number={number} value={value}'

    def switch_tab(self, forward=True):
        current = self.tabs.index('current')
        add = 1 if forward else -1
        _next = (current + add) % self.tabs.index('end')
        self.tabs.select(_next)

    def tab_changed(self, event):
        current_tab = self.tabs.index('current')
        buttons = self.tabs.winfo_children()[current_tab].winfo_children()
        self.activate_button(buttons[0])

    def switch_button(self, diff):
        active = self.active_button
        buttons = active.master.winfo_children()
        total = len(buttons)
        index = buttons.index(active)

        target = (index + diff) % total
        self.activate_button(buttons[target])

    def activate_button(self, button):
        if self.active_button:
            self.active_button.state(['!selected'])
        self.active_button = button
        button.state(['selected'])

    def speak(self, _type, text):
        if _type == "Son":
            filename = self.sounds_folder + '/' + text
            self.speech_pub.publish(filename)
        elif _type == "Texte":
            self.pub.publish(text)

    def battery_callback(self, msg):
        charge_regie = int(self.pattern.findall(check_output("acpi",text=True))[-1].rstrip('%'))

        for label, val in zip(self.charge_labels.values(), [msg.x, msg.y, charge_regie]):
            val = round(val)
            color = 'red' if val < 20 else 'black' if val < 70 else 'green'
            label.config(text=f'{val} %', foreground=color)

    def joystick_callback(self, msg):
        self.joystick_text.set('/!\\ Joystick disconnected /!\\' if not msg.data else '')

    def toggle_face(self):
        self.face_pub.publish(bool(self.face_on.get()))

    def toggle_movement(self):
        self.movement_pub.publish(bool(self.movement_on.get()))

    def go_to_base(self):
        soundThread = threading.Thread(target=self.to_base_thread)
        soundThread.daemon = True
        soundThread.start()

    def to_base_thread(self):
        call(["roslaunch", "kobuki_auto_docking", "activate.launch", "--screen"])

    def debug(self, msg):        
        self.entry.delete("1.0", 'end-1c')
        self.entry.insert("1.0", msg)


if __name__ == '__main__':
    root = tk.Tk()
    root.title('Boite à paroles')
    Gui(root)
    root.mainloop()
