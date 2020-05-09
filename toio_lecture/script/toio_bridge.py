#!/usr/bin/python
# -*- coding: utf-8 -*- 

import rospy
import rosparam
from std_msgs.msg import Int32, Float32, ColorRGBA
from geometry_msgs.msg import Twist, PoseStamped
from toio_lecture.msg import *
from bluepy import btle
import math

SCAN_HANDLE = 13
MOTOR_HANDLE = 17
LAMP_HANDLE = 20
SOUND_HANDLE = 23
MOTION_SENSOR_HANDLE = 26
BUTTON_HANDLE = 30
BATTERY_HANDLE = 34
CONFIG_HANDLE = 38

class MyDelegate(btle.DefaultDelegate):
    def __init__(self, callback):
        btle.DefaultDelegate.__init__(self)
        self.callback = callback

    def handleNotification(self, cHandle, data):
        self.callback(cHandle, data)

class ToioBLE(object):
    def __init__(self, mac_address, iface):
        self.write_buffer = []
        self.WHEEL_RADIUS = 0.0125
        self.WHEEL_DISTANCE = 0.0266
        self.MAT_SIZE = 0.560
        self.POSE_MIN = 45
        self.POSE_MAX = 455
         
        self.peripheral = btle.Peripheral(mac_address, btle.ADDR_TYPE_RANDOM, iface)
        self.peripheral.withDelegate(MyDelegate(self.notify_callback))
        self.set_notify()

    def push_buffer(self, handler, data):
        self.write_buffer.append([handler, data])

    def pop_buffer(self):
        for b in self.write_buffer:
            self.peripheral.writeCharacteristic(b[0], b[1], True)
        self.write_buffer = []

    def __del__(self):
        self.peripheral.disconnect()

    def close(self):
        self.write_motor(0,0)
        self.peripheral.disconnect()
        
    def set_notify(self):
        self.peripheral.writeCharacteristic(SCAN_HANDLE+1, b'\x01', True)
        self.peripheral.writeCharacteristic(BATTERY_HANDLE+1, b'\x01', True)
        self.peripheral.writeCharacteristic(BUTTON_HANDLE+1, b'\x01', True)
        None

    def notify_callback(self, cHandle, data):
        if cHandle == MOTOR_HANDLE and ord(data[0])==1:
            x = ord(data[2]) * 256 + ord(data[1])
            y = ord(data[4]) * 256 + ord(data[3])
            yaw = ord(data[6]) * 256 + ord(data[5])
            print("pos: %i, %i, %i" % (x, y, yaw))
        elif cHandle == BATTERY_HANDLE:
            print("battery: %i" % ord(data[0]))
        elif cHandle == BUTTON_HANDLE:
            print("button: %i" % ord(data[1]))

    def write_motor(self, left, right):
        data1 = [1,1,1,0,2,1,0]
        if int(left) < 0:
            data1[2] = 2
            data1[3] = -int(left)
        else:
            data1[2] = 1
            data1[3] = int(left)

        if int(right) < 0:
            data1[5] = 2
            data1[6] = -int(right)
        else:
            data1[5] = 1
            data1[6] = int(right)

        self.push_buffer(MOTOR_HANDLE, bytearray(data1))

    def write_effect_sound(self, sound_no, volume):
        data = [2, sound_no, volume]
        self.push_buffer(SOUND_HANDLE, bytearray(data))

    def write_midi(self, notes):
        data = [3, 1, len(notes)]
        for note in notes:
            data.append(note.length)
            data.append(note.note)
            data.append(note.volume)
        self.push_buffer(SOUND_HANDLE, bytearray(data))

    def write_color(self, color):
        int_r = int(255 * color.r)
        int_g = int(255 * color.g)
        int_b = int(255 * color.b)
        data = [3, 0, 1, 1, int_r, int_g, int_b]
        self.push_buffer(LAMP_HANDLE, bytearray(data))

    def process(self):
        TIMEOUT = 0.01
        try:
            self.pop_buffer()
            self.peripheral.waitForNotifications(TIMEOUT)
        except:
            rospy.logerr('BLE Error')

class ToioBridge(ToioBLE):
    def __init__(self):
        self.mac_address = rospy.get_param("~address", None)
        self.iface = rospy.get_param("~iface", None)
        self.map_frame = rospy.get_param("~frame_id", "map")
        self.battery_pub = rospy.Publisher('battery', Int32, queue_size=10)
        self.scan_pose_pub = rospy.Publisher('scan_pose', ScanPose, queue_size=10)
        self.pose_pub = rospy.Publisher('pose', PoseStamped, queue_size=10)
        self.twist_sub = rospy.Subscriber("twist", Twist, self.twist_callback)
        self.lamp_sub = rospy.Subscriber('lamp', ColorRGBA, self.lamp_callback)
        self.effect_sound_sub = rospy.Subscriber("sound", Sound, self.sound_callback)
        super(ToioBridge, self).__init__(self.mac_address, self.iface)

    def notify_callback(self, cHandle, data):
        if cHandle == SCAN_HANDLE and ord(data[0])==1:
            scan_pose = ScanPose()
            scan_pose.x = ord(data[2]) * 256 + ord(data[1])
            scan_pose.y = ord(data[4]) * 256 + ord(data[3])
            scan_pose.yaw = ord(data[6]) * 256 + ord(data[5])
            self.scan_pose_pub.publish(scan_pose)

            pose = PoseStamped()
            pose.header.stamp = rospy.get_rostime()
            pose.header.frame_id = self.map_frame
            pose_centor = (self.POSE_MAX + self.POSE_MIN) / 2
            pose_scale = self.MAT_SIZE / (self.POSE_MAX - self.POSE_MIN) / 2
            pose.pose.position.x = (scan_pose.x - pose_centor) * pose_scale
            pose.pose.position.y = -(scan_pose.y - pose_centor) * pose_scale
            z_rad = -scan_pose.yaw / 360.0 * math.pi
            pose.pose.orientation.z = math.sin(z_rad)
            pose.pose.orientation.w = math.cos(z_rad)
            self.pose_pub.publish(pose)

        elif cHandle == BATTERY_HANDLE:
            bat = Int32()
            bat.data = ord(data[0])
            self.battery_pub.publish(bat)

    def twist_callback(self,data):
        left_wheel = data.linear.x / 0.3 *100 - data.angular.z * self.WHEEL_DISTANCE / 2.0 / 0.3 *100 
        right_wheel = data.linear.x / 0.3 *100 + data.angular.z * self.WHEEL_DISTANCE / 2.0 / 0.3 *100 
        self.write_motor(left_wheel, right_wheel)

    def sound_callback(self, sound):
        if sound.mode == Sound.EFFECT_SOUND:
            self.write_effect_sound(sound.effect_id, sound.volume)
        elif sound.mode == Sound.MIDI:
            self.write_midi(sound.notes)

    def lamp_callback(self, color):
        self.write_color(color)

rospy.init_node('toio_bridge')
toio = ToioBridge()
while not rospy.is_shutdown():
    toio.process()
toio.close()
