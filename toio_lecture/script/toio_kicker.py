#!/usr/bin/python
# -*- coding: utf-8 -*- 

import rospy
import rosparam
from std_msgs.msg import Int32, Float32, ColorRGBA
from geometry_msgs.msg import Twist, PoseStamped
from toio_lecture.msg import *
from bluepy import btle
import math


class ToioKicker:
    def __init__(self):
        self.sound_pub = rospy.Publisher('sound', Sound, queue_size=10)
        self.lamp_pub = rospy.Publisher('lamp', ColorRGBA, queue_size=10)
        self.twist_pub = rospy.Publisher('twist', Twist, queue_size=10)
        rospy.sleep(1.0)
        self.pub_sound()
        for i in range(6):
            if i %2 == 0:
                self.turn_red()
                self.twist_left()
            else:
                self.turn_blue()
                self.twist_right()
            rospy.sleep(2.0)
        self.twist_zero()

    def pub_sound(self):
        sound = Sound()
        sound.mode = Sound.MIDI
        sound.notes = [Note() for i in range(7)]
        sound.notes[0].note = Note.C4
        sound.notes[0].volume = 200
        sound.notes[0].length = 40
        sound.notes[1].note = Note.D4
        sound.notes[1].volume = 200
        sound.notes[1].length = 40
        sound.notes[2].note = Note.E4
        sound.notes[2].volume = 200
        sound.notes[2].length = 40
        sound.notes[3].note = Note.F4
        sound.notes[3].volume = 200
        sound.notes[3].length = 40
        sound.notes[4].note = Note.E4
        sound.notes[4].volume = 200
        sound.notes[4].length = 40
        sound.notes[5].note = Note.D4
        sound.notes[5].volume = 200
        sound.notes[5].length = 40
        sound.notes[6].note = Note.C4
        sound.notes[6].volume = 200
        sound.notes[6].length = 40
        self.sound_pub.publish(sound)

    def turn_red(self):
        lamp = ColorRGBA()
        lamp.r = 1
        lamp.g = 0
        lamp.b = 0
        self.lamp_pub.publish(lamp)

    def turn_blue(self):
        lamp = ColorRGBA()
        lamp.r = 0
        lamp.g = 0
        lamp.b = 1
        self.lamp_pub.publish(lamp)
    
    def twist_left(self):
        twist = Twist()
        twist.linear.x = 0.05
        twist.angular.z = -2.0
        self.twist_pub.publish(twist)

    def twist_right(self):
        twist = Twist()
        twist.linear.x = 0.05
        twist.angular.z = 2.0
        self.twist_pub.publish(twist)

    def twist_zero(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.twist_pub.publish(twist)

rospy.init_node('toio_kicker')
ToioKicker()
rospy.spin()    

