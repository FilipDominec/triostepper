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
 * [ ] unify shared .h files between (firmware and backend module)
 * [ ] C-module should raise true Python errors, not just return NULL
 * [ ] end sensors working
   * [x] for each motor separate
   * [ ] test on OWIS linear stages in the lab
   * [ ] somehow avoid locking (end sensors may have hysteresis)
 * [x] command queue
   * [x] bare functionality
   * [x] reliable cyclic queue (to save memory manipulation)
 * [ ] change the board to use IRF7105 for the PWM tool output
 * [ ] change the board to connect Motor3-end_sensor to the "programming" conn
 * [ ] could the backend use openDevice(), and reuse the usbobject through whole python script?
 * [ ] make documentation (this file?)
   * [ ] commands from computer
   * [ ] interpreting the return messages
   * [ ] practical examples of use
   * [ ] firmware overview
   * [ ] board function & building instructions
   * [ ] electrical specifications (for input; for OWIS(R)-compatible D-SUB9 connectors and for the tool)
   * [ ] 
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
 * [ ] make the microstepping lookup table nonlinear (sine/cosine  instead of  linear ramp/constant)
 * [ ] allow slow speed ramp (to avoid step skipping)
 * [ ] enable selecting/finding devices by their ID number (if more than one triostepper is used)
 * [ ] ...
