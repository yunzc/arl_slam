#!/usr/bin/env python

# Yun Chang 2017 

# code to interface with the leddar range sensor used in our case for altitude measurement 

import serial 

ser = serial.Serial('/dev/ttyUSB0', 115200, bytesize=8, parity='N', stopbits=1)

while True:
	a = ser.read()
	a = ''.join(chr(ord(ch) & 0x7f) for ch in a)
	print(a)