#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import sys, tty, termios
import numpy as np
import math
import os, struct, array
from fcntl import ioctl
import sys, tty, termios
import threading


# These constants were borrowed from linux/input.h
AXIS_NAMES = {
    0x00 : 'x',
    0x01 : 'y',
    0x02 : 'z',
    0x03 : 'rx',
    0x04 : 'ry',
    0x05 : 'rz',
    0x06 : 'trottle',
    0x07 : 'rudder',
    0x08 : 'wheel',
    0x09 : 'gas',
    0x0a : 'brake',
    0x10 : 'hat0x',
    0x11 : 'hat0y',
    0x12 : 'hat1x',
    0x13 : 'hat1y',
    0x14 : 'hat2x',
    0x15 : 'hat2y',
    0x16 : 'hat3x',
    0x17 : 'hat3y',
    0x18 : 'pressure',
    0x19 : 'distance',
    0x1a : 'tilt_x',
    0x1b : 'tilt_y',
    0x1c : 'tool_width',
    0x20 : 'volume',
    0x28 : 'misc',
}

BUTTON_NAMES = {
    0x120 : 'trigger',
    0x121 : 'thumb',
    0x122 : 'thumb2',
    0x123 : 'top',
    0x124 : 'top2',
    0x125 : 'pinkie',
    0x126 : 'base',
    0x127 : 'base2',
    0x128 : 'base3',
    0x129 : 'base4',
    0x12a : 'base5',
    0x12b : 'base6',
    0x12f : 'dead',
    0x130 : 'a',
    0x131 : 'b',
    0x132 : 'c',
    0x133 : 'x',
    0x134 : 'y',
    0x135 : 'z',
    0x136 : 'tl',
    0x137 : 'tr',
    0x138 : 'tl2',
    0x139 : 'tr2',
    0x13a : 'select',
    0x13b : 'start',
    0x13c : 'mode',
    0x13d : 'thumbl',
    0x13e : 'thumbr',

    0x220 : 'dpad_up',
    0x221 : 'dpad_down',
    0x222 : 'dpad_left',
    0x223 : 'dpad_right',

    # XBox 360 controller uses these codes.
    0x2c0 : 'dpad_left',
    0x2c1 : 'dpad_right',
    0x2c2 : 'dpad_up',
    0x2c3 : 'dpad_down',
}


#Handle joystick inputs
class Controller():
    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        self.pub = rospy.Publisher("/turtleshow/command", Twist, queue_size=10)
        self.twist = Twist()
        self.twist.linear.y = -1
        self.movement_map = {
            'x': [self.twist.angular, 'z', 1.5],
            'rx': [self.twist.angular, 'z', 2],
            'y': [self.twist.linear, 'x', 0.5],
            'ry': [self.twist.linear, 'x', 1],
        }        
        self.prev_values = {
            'x': 0,
            'rx': 0,
            'y': 0,
            'ry': 0,
        }

        # Iterate over the joystick devices.
        print('Available devices:')

        for fn in os.listdir('/dev/input'):
            if fn.startswith('js'):
                print('  /dev/input/%s' % (fn))

        self.axis_map = []
        self.button_map = []

        # Open the joystick device. Si bug avec js0 mettre js1
        fn = '/dev/input/js0'
        print('Opening %s...' % fn)
        try:
            self.jsdev = open(fn, 'rb')
        except:
            rospy.logerr('No device found')
            self.controller = False
            return
        self.controller = True

        # Get the device name.
        #buf = bytearray(63)
        buf = array.array('B', [0] * 64)
        ioctl(self.jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
        js_name = buf.tostring()
        print('Device name: %s' % js_name)

        # Get number of axes and buttons.
        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a11, buf) # JSIOCGAXES
        num_axes = buf[0]

        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
        num_buttons = buf[0]

        # Get the axis map.
        buf = array.array('B', [0] * 0x40)
        ioctl(self.jsdev, 0x80406a32, buf) # JSIOCGAXMAP

        for axis in buf[:num_axes]:
            axis_name = AXIS_NAMES.get(axis, 'unknown(0x%02x)' % axis)
            self.axis_map.append(axis_name)

        # Get the button map.
        buf = array.array('H', [0] * 200)
        ioctl(self.jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

        for btn in buf[:num_buttons]:
            btn_name = BUTTON_NAMES.get(btn, 'unknown(0x%03x)' % btn)
            self.button_map.append(btn_name)

        print('%d axes found: %s' % (num_axes, ', '.join(self.axis_map)))
        print('%d buttons found: %s' % (num_buttons, ', '.join(self.button_map)))

        while not rospy.is_shutdown():
            self.updatePositionLoop()
            self.pub.publish(self.twist)


    def updatePositionLoop(self):
        if not self.controller:
            return

        # Main event loop
        #while True:
        evbuf = self.jsdev.read(8)
        if evbuf:
            time, value, kind, number = struct.unpack('IhBB', evbuf)

            if kind & 0x80:
                print("(initial)", end=" "),

            if kind & 0x01:
                button = self.button_map[number]
                if button:
                    if value:
                        print("%s pressed" % (button))
                        if button == 'x':
                            self.toggle_autonomy()
                        elif button == 'thumb':
                            self.input_key = 'escape'
                    else:
                        pass
                        print("%s released" % (button))

            if kind & 0x02:
                axis = self.axis_map[number]
                if axis:
                    fvalue = value / 32767.0
                    print(axis, 'value: ', value, ' fvalue: ', fvalue)
                    if axis == 'trottle':
                        print("trottle")
                    elif axis in self.movement_map:
                        obj, attr, scale = self.movement_map[axis]
                        val = getattr(obj, attr) - (fvalue - self.prev_values[axis]) * scale
                        setattr(obj, attr, val)
                        self.prev_values[axis] = fvalue

    def toggle_autonomy(self):
        self.twist.linear.y = -self.twist.linear.y


if __name__ == '__main__':
    controller = Controller()
