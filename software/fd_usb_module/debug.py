#!/usr/bin/env python
#-*- coding: utf-8 -*-

import time, sys, os, traceback
import fd_usb_module 
import numpy as np
import numpy.random as rd

z=2**30
pos = 0 
print fd_usb_module.Move(z+pos, z+pos, z+pos, 768, 768, 768)
time.sleep(1)

p=100000
s=768   

print fd_usb_module.Move(z+p/10, z+p/10, z+p/10, s/2,s/2,s/2)
print fd_usb_module.Move(z+p/5, z+p/5, z+p/5, s*3/4,s*3/4,s*3/4)
print fd_usb_module.Move(z+p, z+p, z+p, s,s,s)

#commandcount = 50
#positions = (rd.rand(commandcount) *  int(sys.argv[1])).astype(int)
#speeds =    (128 + rd.rand(commandcount)*512).astype(int)

#for p,s in zip(positions, speeds):
    #ret = fd_usb_module.Move(z+p, z+p, z+p, s,s,s)
    #print "Command returned: busy=%d, empty=%d, full=%d" % (ret&1, ret&2, ret&4)
    #if ret&4:
        #while fd_usb_module.IsBusy() & 4: 
            #time.sleep(.15); 
            #print ' (queue full, waiting)'
#while fd_usb_module.IsBusy() & 1: 
    #time.sleep(.15); 
    #print ' (device still busy, waiting to end)'

#print fd_usb_module.Move(z+32000, z+32000, z+16000, 32, 32, 64)

