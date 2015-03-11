#!/usr/bin/env python
#-*- coding: utf-8 -*-


import fd_usb_module, time, numpy

zero_position = 2**30    # do not change this unless the firmware is updated as well 
def go(x,y,shutter=False):
    shutterpos = 32 if shutter else 64
    nanoposX = int(x * 256 * 2000)
    nanoposY = int(y * 256 * 2000)
    print nanoposX, nanoposY
    fd_usb_module.Move(zero_position+nanoposX,zero_position+nanoposY,2**30+16*shutterpos,256,256,256)
    time.sleep(.2)
    #fd_usb_module.Move(zero_position+nanoposX,zero_position+nanoposY,2**30+16*shutterpos,512,512,256)
    while fd_usb_module.IsBusy(): print ".",; time.sleep(.2) 

def mkhole(x, y, t):
    go(x,y,shutter=False)
    go(x,y,shutter=True)
    time.sleep(t)
    go(x,y,shutter=False)


def mksieve(x0 = 0, y0 = 0, t=.210, outer_d = 10, spacing = .3):
    holenumber = 0
    oddcolumn = 1
    for dx in numpy.arange(0, outer_d, spacing):
        oddcolumn *=  -1
        for dy in (numpy.arange(0, outer_d, spacing))[::oddcolumn]:
            x, y = (x0 + dx, y0 + dy)
            print "HOLE #%d [%.3f, %.3f], t = %.2f" % (holenumber, x, y, t)
            if (((x0+outer_d/2)-x)**2 + ((y0+outer_d/2)-y)**2) <= (outer_d/2)**2:
                try:
                    mkhole(x, y, t)
                except:
                    time.sleep(1)
                    print "Warning: problem drilling a hole!"
                    mkhole(x, y, t)
                    print "ERROR could not drill this hole!"

