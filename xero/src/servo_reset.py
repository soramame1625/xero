#!/usr/bin/env python

import serial
import time

servo = serial.Serial('/dev/ttyS0', 115200)
time.sleep(0.5)

def torque(ID, data):
    check = 0x00
    txdata = [0xFA, 0xAF, ID, 0x00, 0x24, 0x01, 0x01, data]
    for i in range(2,8):
            check = check^txdata[i]
    txdata.append(check)
    servo.write(txdata)
    time.sleep(0.00025)

def ang_set(ID, ang,speed):
    check = 0x00
    txdata = [0xFA, 0xAF, ID, 0x00, 0x1E, 0x04, 0x01, 0x00FF&ang, 0x00FF&(ang >> 8), 0x00FF&speed, 0x00FF&(speed >> 8)]
    for i in range(2,11):
            check = check^txdata[i]
    txdata.append(check)
    servo.write(txdata)
    time.sleep(0.00025)

torque(0xFF,0x01)
time.sleep(0.1)
ang_set(0xFF,0,200)
time.sleep(0.1)
servo.close
