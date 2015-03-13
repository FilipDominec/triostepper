#!/usr/bin/env python
#-*- coding: utf-8 -*-

import time, sys, os, traceback
import fd_usb_module 

z=2**30
pos = 0 
print fd_usb_module.Move(z+pos, z+pos, z+pos, 256, 256, 256)

pos += int(sys.argv[1])/4
print fd_usb_module.Move(z+pos, z+pos, z+pos, 32, 32, 32)
pos += int(sys.argv[1])/2
print fd_usb_module.Move(z+pos, z+pos, z+pos, 64, 64, 64)
pos += int(sys.argv[1])/2
print fd_usb_module.Move(z+pos, z+pos, z+pos, 128, 128, 128)
pos += int(sys.argv[1])
print fd_usb_module.Move(z+pos, z+pos, z+pos, 256, 256, 256)
pos += int(sys.argv[1])
print fd_usb_module.Move(z+pos, z+pos, z+pos, 256, 256, 256)
pos += int(sys.argv[1])/2
print fd_usb_module.Move(z+pos, z+pos, z+pos, 128, 128, 128)
pos += int(sys.argv[1])/4
print fd_usb_module.Move(z+pos, z+pos, z+pos, 64, 64, 64)
pos += int(sys.argv[1])/8
print fd_usb_module.Move(z+pos, z+pos, z+pos, 32, 32, 32)
pos += int(sys.argv[1])/4
print fd_usb_module.Move(z+pos, z+pos, z+pos, 32, 32, 32)
pos += int(sys.argv[1])/2
print fd_usb_module.Move(z+pos, z+pos, z+pos, 64, 64, 64)
pos += int(sys.argv[1])/2
print fd_usb_module.Move(z+pos, z+pos, z+pos, 128, 128, 128)
pos += int(sys.argv[1])
print fd_usb_module.Move(z+pos, z+pos, z+pos, 256, 256, 256)

pos += int(sys.argv[1])
print fd_usb_module.Move(z+pos, z+pos, z+pos, 256, 256, 256)
pos += int(sys.argv[1])/2
print fd_usb_module.Move(z+pos, z+pos, z+pos, 128, 128, 128)
pos += int(sys.argv[1])/4
print fd_usb_module.Move(z+pos, z+pos, z+pos, 64, 64, 64)
pos += int(sys.argv[1])/8
print fd_usb_module.Move(z+pos, z+pos, z+pos, 32, 32, 32)
pos += int(sys.argv[1])/4
print fd_usb_module.Move(z+pos, z+pos, z+pos, 32, 32, 32)
pos += int(sys.argv[1])/2
print fd_usb_module.Move(z+pos, z+pos, z+pos, 64, 64, 64)
pos += int(sys.argv[1])/2
print fd_usb_module.Move(z+pos, z+pos, z+pos, 128, 128, 128)
pos += int(sys.argv[1])
print fd_usb_module.Move(z+pos, z+pos, z+pos, 256, 256, 256)
pos += int(sys.argv[1])
print fd_usb_module.Move(z+pos, z+pos, z+pos, 256, 256, 256)
pos += int(sys.argv[1])/2
print fd_usb_module.Move(z+pos, z+pos, z+pos, 128, 128, 128)
pos += int(sys.argv[1])/4
print fd_usb_module.Move(z+pos, z+pos, z+pos, 64, 64, 64)
pos += int(sys.argv[1])/8
print fd_usb_module.Move(z+pos, z+pos, z+pos, 32, 32, 32)
pos += int(sys.argv[1])/4
print fd_usb_module.Move(z+pos, z+pos, z+pos, 32, 32, 32)
pos += int(sys.argv[1])/2
print fd_usb_module.Move(z+pos, z+pos, z+pos, 64, 64, 64)
pos += int(sys.argv[1])/2
print fd_usb_module.Move(z+pos, z+pos, z+pos, 128, 128, 128)
pos += int(sys.argv[1])
print fd_usb_module.Move(z+pos, z+pos, z+pos, 256, 256, 256)

pos += int(sys.argv[1])
print fd_usb_module.Move(z+pos, z+pos, z+pos, 256, 256, 256)
pos += int(sys.argv[1])/2
print fd_usb_module.Move(z+pos, z+pos, z+pos, 128, 128, 128)
pos += int(sys.argv[1])/4
print fd_usb_module.Move(z+pos, z+pos, z+pos, 64, 64, 64)
pos += int(sys.argv[1])/8
print fd_usb_module.Move(z+pos, z+pos, z+pos, 32, 32, 32)


#print fd_usb_module.Move(z+32000, z+32000, z+16000, 32, 32, 64)

