/// Following constants must be common for firmware and software
#define ZERO_POSITION (((uint32_t)1)<<30)  // This position is set when the motors come to the end switches; 
										   // it may not be 0 as there are no signed integers used.
										   // Note that number "1" must be promoted to uint32_t _before_ shifting!

#define CMD_MOVE 1
#define CMD_BUFFERED_MOVE 2
#define CMD_SET_DRILL_PWM 3
