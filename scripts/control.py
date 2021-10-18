#!/usr/bin/env python3

import os
import struct
import array
import signal
from fcntl import ioctl
from pathlib import Path
import contextlib
from time import sleep

import rospy
from geometry_msgs.msg import Twist


CONTROLLER_NAME = 'Sony Interactive Entertainment Wireless Controller'

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
        self.device = None
        self.twist = Twist()
        self.twist.linear.y = -1
        self.axis_map = []
        self.button_map = []
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
        signal.signal(signal.SIGINT, self.stop)

    def loop(self):
        while not rospy.is_shutdown():
            while self.device is None:
                self.device = self.find_device()
                if self.device is None:
                    print(f'Waiting for joystick with name {CONTROLLER_NAME}...')
                    sleep(1)
            evbuf = self.get_event()
            if evbuf:
                self.update_position(evbuf)

    def find_device(self):
        for path in Path('/dev/input').glob(r'js[0-9]'):
            buf = bytearray(64)
            try:
                with contextlib.ExitStack() as stack:
                    jsdev = stack.enter_context(path.open('rb'))
                    ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
                    if buf.decode().strip('\0') == CONTROLLER_NAME:
                        self.get_joystick_map(jsdev)
                        print(f'Found joystick {str(path)} with name CONTROLLER_NAME')
                        stack.pop_all() # Keep the file open
                        return jsdev
            except OSError: 
                continue

    def remove_device(self):
        with contextlib.suppress(OSError):
            self.device.close()
        self.device = None

    def get_event(self):
        try:
            evbuf = self.device.read(8)
        except OSError:
            self.remove_device()
            return ''
        return evbuf

    def get_joystick_map(self, jsfile):
        # Get number of axes and buttons.
        buf = bytearray(1)
        ioctl(jsfile, 0x80016a11, buf) # JSIOCGAXES
        num_axes = buf[0]

        buf = bytearray(1)
        ioctl(jsfile, 0x80016a12, buf) # JSIOCGBUTTONS
        num_buttons = buf[0]

        # Get the axis map.
        buf = bytearray(num_axes)
        ioctl(jsfile, 0x80406a32, buf) # JSIOCGAXMAP
        self.axis_map = [AXIS_NAMES.get(axis, f'unknown({hex(axis)})') for axis in buf]

        # Get the button map.
        buf = array.array('H', [0] * num_buttons)
        ioctl(jsfile, 0x80406a34, buf) # JSIOCGBTNMAP
        self.button_map = [BUTTON_NAMES.get(btn, f'unknown({hex(btn)})') for btn in buf]

        print(f'{num_axes} axes found: {", ".join(self.axis_map)}')
        print(f'{num_buttons} buttons found: {", ".join(self.button_map)}')

    def update_position(self, evbuf):
        time, value, kind, number = struct.unpack('IhBB', evbuf)

        if kind & 0x80:
            print('(initial)', end=' '),

        if kind & 0x01:
            button = self.button_map[number]
            if button:
                if value:
                    print(f'{button} pressed')
                    if button == 'x':
                        self.toggle_autonomy()
                    elif button == 'thumb':
                        self.input_key = 'escape'
                else:
                    pass
                    print(f'{button} released')

        if kind & 0x02:
            axis = self.axis_map[number]
            if axis:
                fvalue = value / 32767.0
                print(f'{axis}  value: {value}  fvalue: {fvalue}')
                if axis == 'trottle':
                    print('trottle')
                elif axis in self.movement_map:
                    obj, attr, scale = self.movement_map[axis]
                    val = getattr(obj, attr) - (fvalue - self.prev_values[axis]) * scale
                    setattr(obj, attr, val)
                    self.prev_values[axis] = fvalue

        self.pub.publish(self.twist)

    def toggle_autonomy(self):
        self.twist.linear.y = -self.twist.linear.y

    def stop(self, sig, frame):
        raise KeyboardInterrupt

if __name__ == '__main__':
    try:
        controller = Controller()
        controller.loop()
    except KeyboardInterrupt:
        pass
