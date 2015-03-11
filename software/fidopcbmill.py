#!/usr/bin/env python
#-*- coding: utf-8 -*-
"""
This program reads a G-code file from GEDA PCB program and controls a CNC robot to mill&drill a PCB.

Filip Dominec, dominecf@fzu.cz
"""

# FIXMEs:
#  * somehow enable plotting and avoid app. crashing
#  * (Module) the C module should report error if device not found (now it reports that it is busy)

# future TODOs: 
#  * avoid manual re-positioning of the drill when the CNC is turned on -> calibration using end switches
#  * switch axes: now they are Z, X, Y (MODULE + hardware remake)
#    then check if th X, Y axes have correct signs (MODULE)
#  * function move_if_not_running() (MODULE + use here)
#  * stop_now() for immediate stop (MODULE)
#  * i18n http://svn.majorsilence.com/pygtknotebook/trunk/pygtk-notebook-latest.html#toc-Chapter-10
#  * try this: StatusBar.push(0,"Opened File: " + filename)

# File import roadmap
# Load milling paths
#    1) from .gcode                                .. NO (done, but not available in GUI)
#    2) from .front.gerber (calling pcb2gcode)     .. YES
#       2b) from JH's gerber                          .. NO, as pcb2gcode fails unexpectedly
#    3) from .pcb (calling advoro)                 .. todo by Adam
#    4) from .front.gerber (calling advoro)        .. (todo in future by Adam)
# Load cutting outline
#    1) from .gcode                                .. NO, won't fix
#    2) from .outline.gerber (calling pcb2gcode)   .. NO, fails in a weird way
#    3) from .pcb (calling advoro)                 .. YES?, done and to be tested
#    4) from .front.gerber (calling advoro)        .. (todo in future by Adam)
# Load holes for drilling
#
# The aim: to import either .pcb and have paths+outline at once
#      This requires finishing advoro.
#      (in future: import from Eagle and any .gerber as well.)
# Current workaround: save .pcb and export .gerber, load paths from .gerber, load outline from .pcb

from gi.repository import Gtk      

import time, sys, traceback, signal, re, subprocess, math
import fd_usb_module
#import FDSettingsPickler 

#import scipy
#import matplotlib
## use either "GTKAgg", "GTK" or "GTKCairo" (twice at each line, first occurence with lowercase)
# from matplotlib.backends.backend_gtkagg import FigureCanvasGTKAgg as FigureCanvas    
# from matplotlib.backends.backend_gtkagg import NavigationToolbar2GTKAgg as NavigationToolbar
# fixme: new GTK kept crashing when Matplotlib 
#        was loaded (see http://stackoverflow.com/questions/8109805/understanding-gi-repository)


class Main: 
## ------------------------------ Initialisation ------------------------------
    def __init__(self):#{{{
        ## Initialize device and G-code related variables
        self.automatic_recalculation = False
        self.automatic_device_move = False
        self.DeviceDisabled = False
        self.Paused = False
        self.CommandQueue = []
        self.GcodeExpectsDrillON = False
        self.old_nanoposX = 0
        self.old_nanoposY = 0
        self.old_nanoposZ = 0
        self.zero_position = 2**30    # do not change this unless the firmware is updated as well 

        ## Initialize the graphical user interfase
        self.builder = Gtk.Builder()
        self.builder.add_from_file(sys.path[0] + "/" + "gui.xml") 
        self.builder.connect_signals(self)       
        self.w('window1').connect("destroy", self.on_mainWindow_destroy)
        self.w('window1').connect("key_release_event", self.window1_key_release) ## not done by Gtk.Builder (?)
        self.w('window1').maximize()

        ## Initialize all spinbuttons
        self.w('spbDrillPWM').set_adjustment(Gtk.Adjustment(value=0, lower=0, upper=16, page_size=0, step_increment=1))
        self.w('spbMoveSpeed').set_adjustment(Gtk.Adjustment(value=0, lower=0, upper=256, page_size=0, step_increment=1))
        self.w('spbMillingSpeed').set_adjustment(Gtk.Adjustment(value=0, lower=0, upper=256, page_size=0, step_increment=1))
        for axis in ['X', 'Y', 'Z']:
            self.w('spbMaxNat'+axis).set_adjustment(Gtk.Adjustment(value=1e8, lower=0, upper=1e8, page_size=0, step_increment=256))
            #self.w('spbNatMM'+axis).set_adjustment(Gtk.Adjustment(value=256, lower=0, upper=1e8, page_size=0, step_increment=256))
            self.w('spbNatMM'+axis).set_adjustment(Gtk.Adjustment(value=256, lower=-1e8, upper=1e8, page_size=0, step_increment=256))
            self.w('spbNat'+axis).set_adjustment(Gtk.Adjustment(value=0, lower=-1e8, upper=1e8, page_size=0, step_increment=256))
            for name1 in ['spbRel', 'spbAbs', 'spbOffset']:
                self.w(name1+axis).set_adjustment(Gtk.Adjustment(value=0, lower=-10000, upper=10000, page_size=0, step_increment=1))

        ## Load saved values of spinbuttons
        ## NOTE: this requires the device to be reset always before program is started
        self.SpinButtonList  = ['spbDrillPWM', 'spbMoveSpeed', 'spbMillingSpeed',
                'spbMaxNatX',    'spbMaxNatY',    'spbMaxNatZ',    
                'spbNatMMX',    'spbNatMMY',    'spbNatMMZ',    
                'spbRelX',        'spbRelY',        'spbRelZ']    
        try:
            self.CalibrationFileName = 'RecentCalibration.pickle'
            CalibrationFile = open(self.CalibrationFileName, 'r')
            self.cPickle = __import__('cPickle')    
            ValueList = self.cPickle.load(CalibrationFile)
            for (SpinButton, Value) in zip(self.SpinButtonList, ValueList):
                self.w_setv(SpinButton, Value)
            CalibrationFile.close() 
        except:
            print "Warning: Calibration data could not be loaded. Make sure the device is calibrated properly."


        ## Connect to device
        #try:
            #fd_usb_module = __import__('fd_usb_module')    
        #except:
            #print "Error: Could not load the communication module. Cannot control the device."
            #self.DisableDevice()

        self.automatic_recalculation = True
        self.automatic_device_move = True
        for axis in ['X', 'Y', 'Z']:
            self.w_setv('spbOffset'+axis, -self.w_getv('spbRel'+axis))
        self.w('entStatus').set_text("Idle")
    #}}}
    def w(self, widgetname):#{{{
        """ A shortcut to access widgets """
        return self.builder.get_object(widgetname)    
    #}}}
    def w_setv(self, widgetname, value):#{{{
        """ A helper function that sets the shown value of Gtk.SpinButton """
        self.w(widgetname).get_adjustment().set_value(value)
    #}}}
    def w_getv(self, widgetname):#{{{
        """ A helper function that gets the shown value of Gtk.SpinButton """
        return self.w(widgetname).get_adjustment().get_value()
    #}}}

## ------------------------------ File import and command processing ------------------------------
    def LoadGcode(self, file_name, prevent_duplicate_milling=True): #{{{
        prevent_duplicate_milling = True # XXX
        last_checked_cmd_unit = None
        last_move_omitted = False
        def GetPar(command_string, par_name, coef=1):
            par_search = re.search(par_name + "([\\d.\+-]+)", command_string)
            if par_search: 
                return float(par_search.expand("\\1"))*coef
            else: 
                return None

        ## Read the file content
        gcode_file = open(file_name, 'r') 
        gcode_lines = gcode_file.readlines()
        gcode_file.close() 

        ## Parse the G-code (to the internal "cmd_unit" format)
        variables = {}
        warning_coords_wo_gcode = False
        unit_in_mm = 1   # default units are millimeter
        for (linenumber, line) in enumerate(gcode_lines):
            # NOTE: this code should be put in separate routine to allow the user enter custom G-code lines

            ## Detect variable definition
            VarDef_search = re.search("(#.*)=([^ ]*)", line)        
            if VarDef_search: 
                new_var_name = VarDef_search.expand("\\1")
                new_var_value = VarDef_search.expand("\\2")
                variables[new_var_name] = new_var_value
                continue

            ## Preprocessing: substitute variables
            if re.search("#", line):
                for var_name in variables.keys():
                    line = re.sub(var_name, variables[var_name], line)

            ## Processing the commands 
            ## Note: The command begins with "G" or "M", followed by command number and possibly some parameters
            for cmd_str in filter(lambda s: s!='', re.split("([MG][^M^G]*)", re.sub("[ \t]*", "", line.strip()))):
                cmd_str = cmd_str.strip() + ' '
                if re.match("G", cmd_str) and GetPar(cmd_str, "G")==0:           ## Fast move
                    cmd_unit = ("G0", 
                            GetPar(cmd_str, "X", coef=unit_in_mm), 
                            GetPar(cmd_str, "Y", coef=unit_in_mm), 
                            GetPar(cmd_str, "Z", coef=unit_in_mm))
                elif (re.match("G", cmd_str) and GetPar(cmd_str, "G")==1):       ## Slow move with given milling speed
                    cmd_unit = ("G1", 
                            GetPar(cmd_str, "X", coef=unit_in_mm), 
                            GetPar(cmd_str, "Y", coef=unit_in_mm), 
                            GetPar(cmd_str, "Z", coef=unit_in_mm))
                elif re.match("G20[^\\d]", cmd_str):         ## Set scale of following code to inches
                    unit_in_mm = 25.4
                    continue
                elif re.match("G21[^\\d]", cmd_str):         ## Set scale of following code to millimeters
                    unit_in_mm = 1.0
                    continue
                elif re.match("[XYZ]", cmd_str):            ## If no G-code before coords, do as if line was beginning with "G1"
                    cmd_unit = ("G1", 
                            GetPar(cmd_str, "X", coef=unit_in_mm), 
                            GetPar(cmd_str, "Y", coef=unit_in_mm), 
                            GetPar(cmd_str, "Z", coef=unit_in_mm))
                    warning_coords_wo_gcode +=1
                elif re.match("M3[^\\d]", cmd_str):         ## Drill on 
                    cmd_unit = ("M3",)
                elif re.match("M5[^\\d]", cmd_str):         ## Drill off 
                    cmd_unit = ("M5",)
                elif re.match("[GM][\d]+[^\\d]", cmd_str):      
                    print "Warning: Unrecognized command in G-code at line %d: %s" % (linenumber, cmd_str)
                    continue
                else: 
                    print "Info: Skipping text in G-code at line %d: %s" % (linenumber, cmd_str)
                    continue



                def milling_is_duplicate(last_checked, new, last_mode_omitted):
                    if (new[0] != "G1"):
                        #print " not milling"
                        return False        
                    if new[2] != None and last_checked[2] != None and (new[2]-last_checked[2]) < 0:
                        #print " Y is milling up ->  duplicate!"
                        return True   
                    if new[2] != None and last_checked[2] != None and (new[2]-last_checked[2]) > 0:
                        #print " Y is milling down -> not duplicate"
                        return False  
                    if new[1] != None and last_checked[1] != None:
                        #print " Y is not moving"
                        if  (new[1]-last_checked[1]) > .5:
                            # X is moving left for more than .5 mm -> not duplicate
                            return False
                        elif  (new[1]-last_checked[1]) < -.5:
                            # X is moving right for more than .5 mm -> duplicate!
                            return True
                        else:
                            #Y not moving and X is moving by little step -> be conservative to avoid staircase effect
                            return last_move_omitted   
                    #print " probably not duplicate, fallback"
                    return False    

                if (prevent_duplicate_milling and cmd_unit[0]=="G1") or not last_checked_cmd_unit:
                    if milling_is_duplicate(last_checked_cmd_unit, cmd_unit, last_move_omitted):
                        if not last_move_omitted:
                            self.CommandQueue.append(("G0", None, None, -0.05))
                            self.CommandQueue.append(("G0", None, None, 1.0))            #print "Drill up"
                        last_move_omitted = True
                    else:
                        if last_move_omitted:                                   #print "Drill to start_of_line and down"
                            self.CommandQueue.append(("G0", last_checked_cmd_unit[1], last_checked_cmd_unit[2], 1.0))
                            self.CommandQueue.append(("G1", None, None, -0.05))
                            self.CommandQueue.append(("G1", None, None, 0.0))
                            self.CommandQueue.append(cmd_unit)
                        self.CommandQueue.append(cmd_unit)#print "standard milling"
                        last_move_omitted = False
                else:
                    self.CommandQueue.append(cmd_unit)
                if cmd_unit[0] == "G0": last_move_omitted = False
                last_checked_cmd_unit = cmd_unit
        if warning_coords_wo_gcode:
            print "Warning: coordinates without G-code at %d line(s) total (always interpreted as G1 command)" % warning_coords_wo_gcode
        print "Info: Loaded ", len(self.CommandQueue), "commands from G-code file"

        ## User feedback
        self.w('entQueue').set_text(`len(self.CommandQueue)`)
        while (Gtk.events_pending()): Gtk.main_iteration(); 
    #}}}
    def LoadPCBOutline(self, file_name):#{{{
        ## Extract the PCB size
        ## PCB line format (in 1/100th mil):
	    ##   Line[22000 38000 38000 38000 1000 2000 "clearline"]
        ##        x1    y1    x2    y2    thickness 2x ?
        pcb_file = open(file_name, 'r') 
        pcb_size_line = [line for line in pcb_file.readlines() if ("PCB" in line)][0]
        print pcb_size_line 
        pcb_file.close()
        pcb_size_strings = pcb_size_line.strip('PCB[]" \n').split()
        print pcb_size_strings 
        x_size = float(pcb_size_strings[0])/100*25.4/1000
        y_size = float(pcb_size_strings[1])/100*25.4/1000
        print x_size, y_size


        ## Add the cutting commands
        z_move = 2.0
        self.CommandQueue.append(("G0", None  , None  , z_move))
        self.CommandQueue.append(("G0", 0     , 0     , z_move))
        for z_cut in [0.0, -0.25, -0.5, -0.75, -1.0, -1.25, -1.5]:
            print "z_cut =", z_cut
            self.CommandQueue.append(("G1", 0     , 0     , z_cut))
            self.CommandQueue.append(("G1", 0     , y_size, z_cut))
            self.CommandQueue.append(("G1", x_size, y_size, z_cut))
            self.CommandQueue.append(("G1", x_size, 0     , z_cut))
            self.CommandQueue.append(("G1", 0     , 0     , z_cut))
        self.CommandQueue.append(("G0", 0     , 0     , z_move))
        self.CommandQueue.append(("M3", 0     , 0     , z_move))
        ## TODO real cutting here; use zcut, cut_infeed

        ## User feedback
        self.w('entQueue').set_text(`len(self.CommandQueue)`)
        while (Gtk.events_pending()): Gtk.main_iteration(); 

    #}}}
    def LoadPCBHoles(self, file_name, min_drill = 0.5, min_cutout = 1.0, tool_diam = 0.8, z_move = .5, z_drill = -1.8):#{{{
        pass
        def AddHole(self, x, y, z_move, z_drill):
            self.CommandQueue.append(("G0", x, y, z_move))
            self.CommandQueue.append(("G1", x, y, z_drill))
            self.CommandQueue.append(("G0", x, y, z_move))

        hole_file = open(file_name, 'r') 
        diameter_dict = {}
        ## CNC coordinate format (in 1/10th mil?):
        inch  = 25.4
        scale = inch / 10 / 1000
        holediam = None

        for line in hole_file.readlines():
            line = line.strip()
            TC_match = re.match("T[0-9]+C", line)
            T_match = re.match("T[0-9]+", line)
            XY_match = re.match("X[0-9.]+Y", line)
            if TC_match:
                T = line[(TC_match.start()+1):(TC_match.end()-1)]
                C = line[(TC_match.end()+1):]
                diameter_dict[T] = float(C)*inch
            elif T_match:
                T = line[(T_match.start()+1):(T_match.end())]
                holediam = diameter_dict.get(T, 0)
            elif XY_match:
                X = float(line[(XY_match.start()+1):(XY_match.end()-1)]) * scale
                Y = float(line[(XY_match.end()+1):]) * scale
                print X, Y, holediam
                if holediam >= min_cutout:
                    cut_r = (holediam - tool_diam) / 2
                    cut_dist = 2 * math.pi * cut_r
                    cut_step = tool_diam / 2
                    hole_count = int(cut_dist/cut_step + .5)
                    if hole_count < 4: hole_count = 4
                    #print "hole_count", hole_count
                    
                    for angle_step in range(0, hole_count):
                        angle = float(angle_step) * math.pi*2 /hole_count
                        AddHole(self, 
                                x=X+math.sin(angle)*cut_r, 
                                y=Y+math.cos(angle)*cut_r, 
                                z_move=z_move, z_drill=z_drill)
                elif holediam >= min_drill:
                    #print  "drill", X, Y
                    AddHole(self, x=X, y=Y, z_move=z_move, z_drill=z_drill)
                else:
                    pass
                    #print  "Hole at ", X, Y, "too narrow, not drilled"
        ## User feedback
        self.w('entQueue').set_text(`len(self.CommandQueue)`)
        while (Gtk.events_pending()): Gtk.main_iteration(); 
    #}}}


    def RunCommand(self, cmd_unit):#{{{
        print cmd_unit
        self.automatic_device_move = False
        ## M3, M5 -- drill control commands
        if cmd_unit[0] == "M3":
            self.GcodeExpectsDrillON = True
            self.SetDrillPwm(self.w_getv('spbDrillPWM'))
        if cmd_unit[0] == "M5":
            self.GcodeExpectsDrillON = False
            self.SetDrillPwm(0)

        ## G0, G1
        if cmd_unit[0] in ("G0", "G1"):
            ## Update the new position
            if cmd_unit[1] != None: 
                self.w_setv('spbRelX', cmd_unit[1])
            if cmd_unit[2] != None: 
                self.w_setv('spbRelY', cmd_unit[2])
            if cmd_unit[3] != None: 
                self.w_setv('spbRelZ', cmd_unit[3])
            ## FIXME check if this does not send the message twice

            ## Use different speed for G0 and G1 commands
            speed = self.w_getv('spbMoveSpeed') if (cmd_unit[0] == "G0") else self.w_getv('spbMillingSpeed')
            self.Move(self.w_getv('spbNatX'), self.w_getv('spbNatY'), self.w_getv('spbNatZ'), speed)

        self.automatic_device_move = True

    #}}}

## ------------------------------ Device control ------------------------------
    def Move(self, nanoposX, nanoposY, nanoposZ, speed_mm_s):#{{{
        if self.DeviceDisabled: return
        pwm_freq = 488

        ## Calculate the speed per axis (for diagonal movement)
        differenceX_mm = (self.old_nanoposX - nanoposX) / self.w_getv('spbNatMMX')
        differenceY_mm = (self.old_nanoposY - nanoposY) / self.w_getv('spbNatMMY')
        differenceZ_mm = (self.old_nanoposZ - nanoposZ) / self.w_getv('spbNatMMZ')
        total_distance_mm = (differenceX_mm**2 + differenceY_mm**2 + differenceZ_mm**2)**.5

        ## Prevent division by zero
        if total_distance_mm == 0: return 

        #print ' differenceX_mm/total_distance_mm = ', differenceX_mm/total_distance_mm 
        nanospeedX = int(abs(differenceX_mm * self.w_getv('spbNatMMX') * speed_mm_s / total_distance_mm / pwm_freq))
        nanospeedY = int(abs(differenceY_mm * self.w_getv('spbNatMMY') * speed_mm_s / total_distance_mm / pwm_freq))
        nanospeedZ = int(abs(differenceZ_mm * self.w_getv('spbNatMMZ') * speed_mm_s / total_distance_mm / pwm_freq))
        if (nanospeedX == 0): nanospeedX += 1
        if (nanospeedY == 0): nanospeedY += 1
        if (nanospeedZ == 0): nanospeedZ += 1

        ## Send the command to the device
        attempts_left = 30

        while fd_usb_module.IsBusy(): 
            #print "Waiting, robot is busy or not responding"
             #TODO (Module) the C module should report error if device not found (now it reports that it is busy)
            while (Gtk.events_pending()): Gtk.main_iteration(); 
            self.w('entStatus').set_text("Busy")
            time.sleep(.1)
        # todo: when the device has correct motor order (and axes direction), make sure this call matches it!
        #print(int(nanoposZ+self.zero_position), 
                #int(nanoposX+self.zero_position), 
                #int(-nanoposY+self.zero_position), 
                #nanospeedZ,
                #nanospeedX, 
                #nanospeedY 
                #)
        fd_usb_module.Move(
                int(nanoposZ+self.zero_position), 
                int(nanoposX+self.zero_position), 
                int(-nanoposY+self.zero_position), 
                nanospeedZ,
                nanospeedX, 
                nanospeedY 
                )
        self.w('entStatus').set_text("Idle")
        ## Update the old position and exit the routine
        self.old_nanoposX = nanoposX
        self.old_nanoposY = nanoposY
        self.old_nanoposZ = nanoposZ
    #}}}
    def SetDrillPwm(self, drill_pwm):#{{{
        print "DRILL", drill_pwm
        if self.DeviceDisabled: return
        ## Send the command to the device
        attempts_left = 30
        while attempts_left > 0:
            try:
                fd_usb_module.SetDrillPwm(int(drill_pwm))
                return
            except:
                print "Error: DrillPWM command failed"
                attempts_left -= 1
                time.sleep(.1)

        ## If communication failed every time, disable any device movements
        self.DisableDevice()
    #}}}
    def DisableDevice(self):#{{{
        """ If communication failed every time, report error and disable any device movements """
        print "Error: sorry, device did not respond. Job aborted to prevent misinterpretation. Check connection and restart the program."
        self.w('entStatus').set_text("Error - Disabled")
        self.DeviceDisabled = True
    #}}}

## ------------------------------ GUI event handlers ------------------------------
    def set_ETA(self):#{{{
        self.w('entQueue').set_text(`len(self.CommandQueue)`)
        try:
            pass
        except: 
            pass
#}}}
    def window1_key_press(self,sender, what):#{{{
        if what.get_keycode()[1] == 37:
            step = .1
        elif what.get_keycode()[1] == 50:
            step = .01
        else: 
            return
        for axis in ['X', 'Y', 'Z']:
                self.w('spbRel'+axis).get_adjustment().set_step_increment(step)
                self.w('spbOffset'+axis).get_adjustment().set_step_increment(step)
        #print what.get_keycode()[1] == 37:
        #print what.get_keyval()[1] % 256
#}}}
    def window1_key_release(self,sender, what):#{{{
        #print "UP", what.get_keycode()[1]
        for axis in ['X', 'Y', 'Z']:
            self.w('spbRel'+axis).get_adjustment().set_step_increment(1.)
            self.w('spbOffset'+axis).get_adjustment().set_step_increment(1.)
#}}}
    def on_mainWindow_destroy(self, widget):#{{{
        ## Store saved values of spinbuttons
        #try:
        CalibrationFile = open(self.CalibrationFileName, 'w')
        ValueList = []
        for SpinButton in self.SpinButtonList:
            ValueList.append(self.w_getv(SpinButton))
            print SpinButton
        self.cPickle.dump(ValueList, CalibrationFile)
        CalibrationFile.close() 
        #except:
            #print "Calibration data could not be saved."

        ## Finally end the application
        Gtk.main_quit()
    #}}}
    def on_signal(self, signum, frame):#{{{
        print 'on_signal', signum
        Gtk.main_quit()
    #}}}
    def spbNat_valuechanged(self, sender):#{{{
        """ Sets new value of the relative and absolute coordinates (Rel = Abs - Offset) 
        and moves the device to the new position immediately """
        ## This routine may be switched off
        if not self.automatic_recalculation: return
        for axis in ['X', 'Y', 'Z']:
            if sender is self.w('spbNat'+axis):
                self.w_setv('spbAbs'+axis, self.w_getv('spbNat'+axis)   / self.w_getv('spbNatMM'+axis))
                self.w_setv('spbRel'+axis, self.w_getv('spbNat'+axis)   / self.w_getv('spbNatMM'+axis) - self.w_getv('spbOffset'+axis))

        if self.automatic_device_move:
            self.Move(self.w_getv('spbNatX'), self.w_getv('spbNatY'), self.w_getv('spbNatZ'), self.w_getv('spbMoveSpeed'))
        
    #}}}
    def spbAbs_valuechanged(self, sender):#{{{
        """ Sets new value of the relative coordinate (Rel = Abs - Offset) and moves the device to the new position immediately """
        if not self.automatic_recalculation: return
        for axis in ['X', 'Y', 'Z']:
            if sender is self.w('spbAbs'+axis):
                self.w_setv('spbRel'+axis, self.w_getv('spbAbs'+axis) - self.w_getv('spbOffset'+axis))
                self.w_setv('spbNat'+axis, self.w_getv('spbAbs'+axis) * self.w_getv('spbNatMM'+axis))
    #}}}
    def spbRel_valuechanged(self, sender):#{{{
        """ Sets new value of the absolute coordinate (Abs = Rel + Offset) and moves the device to the new position immediately """
        if not self.automatic_recalculation: return
        for axis in ['X', 'Y', 'Z']:
            if sender is self.w('spbRel'+axis):
                self.w_setv('spbAbs'+axis, self.w_getv('spbRel'+axis) + self.w_getv('spbOffset'+axis))
                self.w_setv('spbNat'+axis, (self.w_getv('spbRel'+axis) + self.w_getv('spbOffset'+axis))*self.w_getv('spbNatMM'+axis))
    #}}}
    def spbOffset_valuechanged(self, sender):#{{{
        """ Sets new value of the absolute coordinate (Abs = Rel + Offset) """
        if not self.automatic_recalculation: return
        for axis in ['X', 'Y', 'Z']:
            if sender is self.w('spbOffset'+axis):
                self.w_setv('spbAbs'+axis, self.w_getv('spbRel'+axis) + self.w_getv('spbOffset'+axis))
                self.w_setv('spbNat'+axis, (self.w_getv('spbRel'+axis) + self.w_getv('spbOffset'+axis))*self.w_getv('spbNatMM'+axis))
    #}}}
    def spbMaxNat_valuechanged(self, sender):#{{{
        """ Adjusts the limit for the spbNatXYZ components so that the device never crashes its limits """
        if not self.automatic_recalculation: return
        for axis in ['X', 'Y', 'Z']:
            if sender is self.w('spbMaxNat'+axis):
                self.w('spbRel'+axis).get_adjustment().set_upper(self.w_getv('spbMaxNat'+axis))
    #}}}
    def spbNatMM_valuechanged(self, sender):#{{{
        """ For calibration: Sets the new value of all positions that are given in millimeters (i. e. Abs, Rel). """
        if not self.automatic_recalculation: return
        # todo: think up: if NatMM changes, shall the native units stay constant and the position in millimetets change, or vice versa?
        for axis in ['X', 'Y', 'Z']:
            if sender is self.w('spbNatMM'+axis):
                self.w_setv('spbAbs'+axis, self.w_getv('spbNat'+axis)   / self.w_getv('spbNatMM'+axis))
                self.w_setv('spbRel'+axis, self.w_getv('spbNat'+axis)   / self.w_getv('spbNatMM'+axis) - self.w_getv('spbOffset'+axis))
    #}}}
    def spbDrillPWM_valuechanged(self, sender):#{{{
        if self.automatic_device_move:
            if self.w('chbDrillAlwaysON').get_active() or self.GcodeExpectsDrillON:
                self.SetDrillPwm(self.w_getv('spbDrillPWM'))
    #}}}
    def chbDrillAlwaysON_toggled(self, sender):#{{{
        if self.automatic_device_move:
            if self.w('chbDrillAlwaysON').get_active() or self.GcodeExpectsDrillON:
                self.SetDrillPwm(self.w_getv('spbDrillPWM'))
            else:
                self.SetDrillPwm(0)
    #}}}
    def btnRestoreAbsCoords_clicked(self, sender):#{{{
        ## TODO not implemented yet
        pass
        #}}}
    def btnNewCommand_clicked(self, sender):#{{{
        # TODO
        pass
    #}}}
    def btnRunOneCommand_clicked(self, sender):#{{{
        if self.CommandQueue != []:
            ## Take one command out of self.CommandQueue
            cmd_unit = self.CommandQueue[0]
            self.CommandQueue = self.CommandQueue[1:] 

            ## Execute the command
            self.RunCommand(cmd_unit)

            ## Update GUI
            self.w('entQueue').set_text(`len(self.CommandQueue)`)
    #}}}
    def btnRunAllCommands_clicked(self, sender):#{{{
        while self.CommandQueue != []:
            while self.Paused:
                if fd_usb_module.IsBusy():
                    self.w('entStatus').set_text("Paused - Busy")
                else:
                    self.w('entStatus').set_text("Paused - Idle")
                while (Gtk.events_pending()): Gtk.main_iteration(); 
                time.sleep(.2)

            ## Take one command out of self.CommandQueue
            cmd_unit = self.CommandQueue[0]
            self.CommandQueue = self.CommandQueue[1:] 

            ## Execute the command
            self.RunCommand(cmd_unit)

            ## Update GUI
            self.w('entQueue').set_text(`len(self.CommandQueue)`)
            while (Gtk.events_pending()): Gtk.main_iteration(); 
        self.w('entStatus').set_text("Idle")
    #}}}
    def btnResetRelativeCoords_clicked(self, sender):#{{{
        self.automatic_device_move = False
        for axis in ['X', 'Y', 'Z']:
            self.automatic_recalculation = False
            self.w_setv('spbOffset'+axis,  self.w_getv('spbOffset'+axis) + self.w_getv('spbRel'+axis))
            self.automatic_recalculation = True ## FIXME
            self.w_setv('spbRel'+axis, 0)
        self.automatic_device_move = True
    #}}}
    def runBash(self, cmd): #{{{
        p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
        out = p.stdout.read().strip()
        return out
    #}}}
    def btnPause_clicked(self, sender):#{{{
        self.Paused = not self.Paused
        if self.Paused:
            self.w('btnPause').set_label('Continue')
        else:
            self.w('btnPause').set_label('Pause now')
        #}}}
    def btnStop_clicked(self, sender):#{{{
        self.CommandQueue = []

        ## Update GUI
        self.w('entQueue').set_text(`len(self.CommandQueue)`)
        while (Gtk.events_pending()): Gtk.main_iteration(); 
    #}}}
    def add_filters(self, dialog, name_patterns):#{{{
        for name_pattern in name_patterns:
            filter = Gtk.FileFilter()
            filter.set_name(name_pattern[0])
            filter.add_pattern(name_pattern[1])
            dialog.add_filter(filter)
            dialog.set_filter(filter)
            #}}}
    def btnLoadGerber_clicked(self, sender):#{{{
        ## Initialize file chooser dialog 
        dialog = Gtk.FileChooserDialog(title="Select the gerber file containing the solder side layout", 
                buttons=(Gtk.STOCK_OK, Gtk.ResponseType.OK, Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL)) 
        self.add_filters(dialog, [('All files', '*.*'), ('All Gerber files', '*.gbr'), ('Gerber front side', '*.front.gbr')])

        ## Run the dialog
        dialog_response = dialog.run()
        gerber_name = dialog.get_filename()
        dialog.destroy()
        if dialog_response == Gtk.ResponseType.OK:
            ## Run the pcb2gcode command
            print gerber_name
            comparams = " --metric --dpi 300 --zwork 0.0 --zsafe 2 --zchange 10 "
            try:
                command = "pcb2gcode --front "+gerber_name+" --mill-feed 1 --offset 3.0 "+comparams+ \
                        " --mill-speed 1 --front-output "+gerber_name+".gcode"
                print command
                cmd_report = self.runBash(command)
                ## Import the milling paths
                self.LoadGcode(gerber_name+".gcode")
            except:
                print "Error:\tConversion with pcb2gcode failed; nothing imported!"
                raise
                return
    #}}}
    def btnLoadPCBOutline_clicked(self, sender):#{{{
        ## Initialize file chooser dialog 
        dialog = Gtk.FileChooserDialog(title="Select the .PCB file to retrieve the outline", 
                buttons=(Gtk.STOCK_OK, Gtk.ResponseType.OK, Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL)) 
        self.add_filters(dialog, [('All files', '*.*'), ('All PCB files', '*.pcb')])

        ## Run the dialog
        dialog_response = dialog.run()
        pcb_name = dialog.get_filename()
        dialog.destroy()

        if dialog_response == Gtk.ResponseType.OK:
            try:
                self.LoadPCBOutline(pcb_name)
            except:
                print "Error:\tOutline could not be retrieved, no command added!"
                raise
                return
    #}}}

    def btnLoadPCBHoles_clicked(self, sender): #{{{
        ## Initialize file chooser dialog 
        dialog = Gtk.FileChooserDialog(title="Select the .CNC file to read the drill locations", 
                buttons=(Gtk.STOCK_OK, Gtk.ResponseType.OK, Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL)) 
        self.add_filters(dialog, [('All files', '*.*'), ('All CNC files', '*.cnc')])

        ## Run the dialog
        dialog_response = dialog.run()
        cnc_name = dialog.get_filename()
        dialog.destroy()

        if dialog_response == Gtk.ResponseType.OK:
            try:
                self.LoadPCBHoles(cnc_name)  ## todo drill diameters&options here?
            except:
                print "Error:\tHoles could not be retrieved, no command added!"
                raise
                return
    #}}}


def signal_handler (*args):
    print "SIGNAL:", args
    sys.exit()

if __name__ == '__main__':        # avoid execution if loaded as a module only
    try:
        MainInstance = Main()
        signal.signal(signal.SIGINT, signal.SIG_DFL)                # Enables keyboard Ctrl+C interrupt of running GUI
        MainInstance.w("window1").show_all()
        Gtk.main()
        #http://slavino.sk/programovanie/python/256-ukoncovanie-gtk-aplikacii
    except:
        traceback.print_exc()
        #print >>sys.stderr
        #print >>sys.stderr, "Press Enter to exit."
        #blah = raw_input()    # enables to read the error report on volatile terminal


