#!/usr/bin/python
# -*- coding: utf-8 -*- 

from bluepy import btle
import sys

LAMP_HANDLE = 20
BATTERY_HANDLE = 34

if len(sys.argv) != 2:
    print("1 argument (mac address of toio) is nessesary")
    sys.exit( )

# connect
toio_peripheral = btle.Peripheral(sys.argv[1], btle.ADDR_TYPE_RANDOM, 1)

# read battery status
print("battery", ord(toio_peripheral.readCharacteristic(BATTERY_HANDLE)))

# write lamp
data = [3, 0, 1, 1, 200, 50, 50]
toio_peripheral.writeCharacteristic(LAMP_HANDLE, bytearray(data), True)
