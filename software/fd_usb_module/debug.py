#!/usr/bin/env python
#-*- coding: utf-8 -*-

import time, sys, os, traceback
import fd_usb_module 

z=2**30
pos = int(sys.argv[1])
print fd_usb_module.Move(z+pos, z+pos, z+pos, 32, 32, 32)
#print fd_usb_module.Move(z, z, z, 32, 32, 32)
#print fd_usb_module.Move(z+32000, z+32000, z+16000, 32, 32, 64)

