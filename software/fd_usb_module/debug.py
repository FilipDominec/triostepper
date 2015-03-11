#!/usr/bin/env python
#-*- coding: utf-8 -*-

import time, sys, os, traceback
import fd_usb_module 

z=2**30
print fd_usb_module.Move(z+32000, z+32000, z, 32, 32, 32)

