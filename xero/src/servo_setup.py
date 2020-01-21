#!/usr/bin/env python

import serial
import time

servo = serial.Serial('/dev/ttyAMA0', 115200)
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

def m_save(ID):
    check = 0x00
    txdata = [0xFA, 0xAF, ID, 0x40, 0xFF, 0x00, 0x00]
    for i in range(2,7):
            check = check^txdata[i]
    txdata.append(check)
    servo.write(txdata)
    time.sleep(0.00025)

def reverse(ID, data):
    check = 0x00
    txdata = [0xFA, 0xAF, ID, 0x00, 0x05, 0x01, 0x01, data]
    for i in range(2,8):
            check = check^txdata[i]
    txdata.append(check)
    servo.write(txdata)
    time.sleep(0.00025)

reverse(22,0)
time.sleep(0.1)
m_save(22)
time.sleep(0.1)
servo.close
