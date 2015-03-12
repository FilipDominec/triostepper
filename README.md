# Triostepper overview
## 
Simple ATMega8-based controller for three stepper motors
* with USB 1.1 interface
* unipolar/bipolar motor support
* microstepping 
* and drill PWM control


# Binary communication interface
## Commands from computer
## Status byte format

# TODOs
 * [ ] end sensors working
   * [x] for each motor separate
   * [ ] test on OWIS linear stages in the lab
   * [ ] somehow avoid locking (end sensors may have hysteresis)
 * [ ] command buffer
   * [ ] bare functionality
   * [ ] reliable cyclic buffer (to save memory manipulation - have done, but where?)
 * [ ] change the board to use IRF7105 for the output
 * [ ] change the board to connect Motor3-end_sensor to the "programming" conn
 * [ ] implement all commands
   * [ ] CMD_DWELL 	
   * [ ] CMD_READPOS 
   * [ ] CMD_RESETPOS
   * [ ] CMD_ABORT 	
   * [ ] CMD_BUFDUMP 	
   * [ ] CMD_READ_MY_ID 
   * [ ] CMD_WRITE_MY_ID
   * [ ] CMD_I2C_TRANSMIT
   * [ ] CMD_I2C_RECEIVE
 * [ ] slow speed ramp (to avoid step skipping)
