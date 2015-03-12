/// Following constants must be common for firmware and software

#define ZERO_POSITION (((uint32_t)1)<<30)  // This position is the "zero" (there are no signed integers used)
					   // Note that number 1 must be promoted to uint32_t _before_ shifting!


// commands 1-16 are buffered
#define CMD_MOVE 		1

// TODO
#define CMD_DWELL 		2

#define CMD_SET_DRILL_PWM 	3

// TODO
#define CMD_I2C_TRANSMIT	4
// TODO
#define CMD_I2C_RECEIVE		5


// commands over 16 are immediate
// TODO
#define CMD_BUFDUMP 		16
// TODO
#define CMD_READ_MY_ID 		17
// TODO
#define CMD_WRITE_MY_ID		18
// TODO
#define CMD_ABORT 		19
// TODO
#define CMD_RESETPOS 		20
// TODO
#define CMD_READPOS 		21

