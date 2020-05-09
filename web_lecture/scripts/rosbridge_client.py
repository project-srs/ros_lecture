#!/usr/bin/env python
# -*- coding:utf8 -*-
import time
from roslibpy import Message, Ros, Topic
import time

class rosbridge_client:
    def __init__(self):
        self.ros_client = Ros('127.0.0.1', 9090)
        print("wait for server")
        self.publisher = Topic(self.ros_client, '/cmd_vel', 'geometry_msgs/Twist')
        self.listener = Topic(self.ros_client, '/odom', 'nav_msgs/Odometry')
        self.listener.subscribe(self.callback)

        self.ros_client.on_ready(self.start_thread, run_in_thread=True)
        self.ros_client.run_forever()

    def callback(self, message):
        x = message["pose"]["pose"]["position"]["x"]
        y = message["pose"]["pose"]["position"]["y"]
        print(x, y)

    def start_thread(self):
        while True:
            if self.ros_client.is_connected:
                self.publisher.publish(Message({
                    'linear': {
                        'x': 0.5,
                        'y': 0,
                        'z': 0
                    },
                    'angular': {
                        'x': 0,
                        'y': 0,
                        'z': 0.5
                    }
                }))
            else:
                print("Disconnect")
                break
            time.sleep(1.0)

if __name__ == '__main__':
    rosbridge_client()