#!/usr/bin/python
# -*- coding: utf-8 -*- 

from bluepy import btle
import sys

BATTERY_HANDLE = 34

class MyDelegate(btle.DefaultDelegate):
    def __init__(self):
        btle.DefaultDelegate.__init__(self)
        print("set delegate")

    def handleNotification(self, cHandle, data):
        print("callback")
        if(cHandle == BATTERY_HANDLE):
            print("battery", ord(data))

if len(sys.argv) != 2:
    print("1 argument (mac address of toio) is nessesary")
    sys.exit( )

# connect
toio_peripheral = btle.Peripheral(sys.argv[1], btle.ADDR_TYPE_RANDOM, 1)

# set delegate
toio_peripheral.withDelegate(MyDelegate())

# set notify
toio_peripheral.writeCharacteristic(BATTERY_HANDLE+1, b'\x01', True)

try:
    while True:
        TIMEOUT = 0.1
        toio_peripheral.waitForNotifications(TIMEOUT)
except KeyboardInterrupt:
    None
