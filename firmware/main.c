/* Name: main.c
 * Project: FiDoCNC 
 * Atmega8 fuses: 0xff 0xd9
 * Description:
 *    The following code is the firmware for Atmega8A, which serves as a stepper motor controller for a simple CNC. 
 *    Features:
 *    		- minimum electronics (cost < $4) 
 *    		- triple stepper motor driver with PWM microstepping 
 *    		- USB communication in native binary format; G-code/DXF/SVG interpreter in computer
 *    		- PWM drill control included
 * Author: Christian Starkjohann, Filip Dominec
 * Creation Date: 2011-09-07
 * Tabsize: 4
 * Copyright:   (c) 2011-2013 by Filip Dominec
 * 				(c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH for the USB routine
 * License: GNU GPL v2 (see License.txt)
 */

#define	 F_CPU 16000000UL
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>	 /* for _delay_ms() */
#include <avr/eeprom.h>

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv/usbdrv.h"
#include "usbdrv/oddebug.h"	
#include "signal_definition.h"	


/*{{{ USB interface*/
/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

PROGMEM const char usbHidReportDescriptor[22] = {	/* USB report descriptor */
	0x06, 0x00, 0xff,			  // USAGE_PAGE (Generic Desktop)
	0x09, 0x01,					// USAGE (Vendor Usage 1)
	0xa1, 0x01,					// COLLECTION (Application)
	0x15, 0x00,					//   LOGICAL_MINIMUM (0)
	0x26, 0xff, 0x00,			  //   LOGICAL_MAXIMUM (255)
	0x75, 0x08,					//   REPORT_SIZE (8)
	0x95, 0x80,					//   REPORT_COUNT (128)
	0x09, 0x00,					//   USAGE (Undefined)
	0xb2, 0x02, 0x01,			  //   FEATURE (Data,Var,Abs,Buf)
	0xc0						   // END_COLLECTION
};
/* Since we define only one feature report, we don't use report-IDs (which
 * would be the first byte of the report). The entire report consists of 128
 * opaque data bytes.
 */
/*}}}*/
// pins definition /*{{{*/
/* 	M1: C0-C3
	M2: B0-B3
	M3: D4-D7
	Sensors: B4 B5 D0
	Reserved: 
	  C4 C5  (i2c), D2 D1 (USB), D3 (int1)
	  B6 B7 (crystal), C6 (reset)
	  */
//   Motor 0 at PORTC 
#define M1aP  	_BV(PC0)   // first motor, first coil
#define M1aN  	_BV(PC1)
#define M1bP  	_BV(PC2)   // first motor, second coil
#define M1bN  	_BV(PC3)
#define MOTOR1_PINS (M1aP | M1aN | M1bP | M1bN)
#define MOTOR1_DDR DDRC
#define MOTOR1_PORT PORTC
//   Motor 2 at PORTB
#define M2aP  	_BV(PB0)   // second motor, first coil
#define M2aN  	_BV(PB1)
#define M2bP  	_BV(PB2)   // second motor, second coil
#define M2bN  	_BV(PB3)
#define MOTOR2_PINS (M2aP | M2aN | M2bP | M2bN)
#define MOTOR2_DDR DDRB
#define MOTOR2_PORT PORTB
//   Motor 3 at PORTD 
#define M3aP  	_BV(PD4)   // third motor, first coil
#define M3aN  	_BV(PD5)
#define M3bP  	_BV(PD6)   // third motor, second coil
#define M3bN  	_BV(PD7)
#define MOTOR3_PINS (M3aP | M3aN | M3bP | M3bN)
#define MOTOR3_DDR DDRD
#define MOTOR3_PORT PORTD
/// End-sensors
#define SETUP_END_SENSORS {PORTB |= _BV(PB4); PORTB |= _BV(PB5); PORTD |= _BV(PD0); }
#define AT_END_SENSOR1 ((PINB & _BV(PB4)) == 0)
#define AT_END_SENSOR2 ((PINB & _BV(PB5)) == 0)
#define AT_END_SENSOR3 ((PIND & _BV(PD0)) == 0)

/// Drill PWM control
#define DRILL_PWM_PORT PORTC	
#define DRILL_PWM_DDR  DDRB
#define DRILL_PWM_PIN  _BV(PC5)
/*}}}*/

/// Following global variables concern communication etc.
#define USB_INPUT_BUFFER_LENGTH 32
uint8_t usb_input_buffer[USB_INPUT_BUFFER_LENGTH];
uint8_t usb_input_bufptr;

#define MAX_CMD_LENGTH 32
#define MAX_CMD_COUNT 16
uint8_t cmdbuf[MAX_CMD_LENGTH*MAX_CMD_COUNT];
uint8_t cmdbufS;
uint8_t cmdbufE;


// Peripheral control variables/*{{{*/
/// Following global variables/constants are common for all motors
#define STEPS (uint8_t)8
#define MICROSTEP_PER_STEP (uint32_t)16
#define NANOSTEP_PER_MICROSTEP (uint32_t)16
#define NANOSTEP_PER_STEP (uint32_t)(MICROSTEP_PER_STEP*NANOSTEP_PER_MICROSTEP)
volatile static uint32_t nanospeed_max;		// maximum speed value that the tool may move (even diagonally)
volatile static uint8_t drill_pwm;			// maximum drill speed
volatile uint8_t timer_interrupt_occured;	// set by timer ISR, initiates a motor state change etc.
volatile uint8_t global_pwm_counter;		// counts timer interrupts, determines when a new PWM cycle shall begin


/// Following global variables are separate for each motor
volatile char status[3]; 		
#define STATUS_IDLE 0
#define STATUS_RUNNING 1

volatile static uint32_t nanopos[3];	// stores the motor position with bit depth sufficient not only for PWM "microsteps"
				 	// but also for PWM speed control "nanosteps"
volatile static uint32_t real_step[3]; 		//   |-->  calculated base step of motor
volatile static uint8_t  real_pwm[3];		//   '-->  calculated PWM value
volatile static uint32_t target_nanopos[3];	// motor position that shall be reached (usually set by incoming command)
volatile static uint32_t nanospeed[3];		// value that increments/decrements the real_nanopos each PWM cycle
/*}}}*/
const uint16_t lookup_table[MICROSTEP_PER_STEP+1] = {/*     MOTOR CONTROL{{{*/
		0b0000000000000000,
		0b1000000000000000,
		0b1000000010000000,
		0b1000010000010000,
		0b1000100010001000,
		0b1000100101001000,
		0b1001010010010100,
		0b1001010101010100,
		0b1010101010101010,
		0b0110101010101011,
		0b0110101101101011,
		0b0111011010110111,
		0b0111011101110111,
		0b0111101111101111,
		0b0111111101111111,
		0b0111111111111111,
};
const uint8_t MOTOR1_STATES[STEPS] = {M1aN | M1bN, M1aN       , M1aN | M1bP,       M1bP, M1aP | M1bP, M1aP       , M1aP | M1bN, M1bN };
const uint8_t MOTOR2_STATES[STEPS] = {M2aN | M2bN, M2aN       , M2aN | M2bP,       M2bP, M2aP | M2bP, M2aP       , M2aP | M2bN, M2bN };
const uint8_t MOTOR3_STATES[STEPS] = {M3aN | M3bN, M3aN       , M3aN | M3bP,       M3bP, M3aP | M3bP, M3aP       , M3aP | M3bN, M3bN };
/*}}}*/
// Simple half stepping using 4 transistors (for unipolar motor) or 2 H-bridges (for bipolar motor).
// In bipolar motor, M0aP and M0aN control the first coil polarity, M0bP and M0bN control the second coil.
// Note: M0aP and M0aN may never be switched on simultaneously. The same for M0bP and M0bN.
void inline set_motor1_step(uint32_t step)/*{{{*/
{
	MOTOR1_PORT &= ~(MOTOR1_PINS & (~MOTOR1_STATES[(step % STEPS)]));
	MOTOR1_PORT |= MOTOR1_STATES[(step % STEPS)];
}/*}}}*/
void inline set_motor2_step(uint32_t step)/*{{{*/
{
	MOTOR2_PORT &= ~(MOTOR2_PINS & (~MOTOR2_STATES[(step % STEPS)]));
	MOTOR2_PORT |= MOTOR2_STATES[(step % STEPS)];
}/*}}}*/
void inline set_motor3_step(uint32_t step)/*{{{*/
{
	MOTOR3_PORT &= ~(MOTOR3_PINS & (~MOTOR3_STATES[(step % STEPS)]));
	MOTOR3_PORT |= MOTOR3_STATES[(step % STEPS)];
}/*}}}*/

ISR(TIMER0_OVF_vect)  // TIMER0 interrupt service routine/*{{{*/
{                  
	timer_interrupt_occured = 1;
};/*}}}*/

usbMsgLen_t usbFunctionSetup(uint8_t data[8])/*{{{*/
{
usbRequest_t	*rq = (void *)data;
	if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){	/* HID class request */
		if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
			// TODO now the reading starts
			for (usb_input_bufptr=0; usb_input_bufptr<USB_INPUT_BUFFER_LENGTH; usb_input_bufptr++) 
					usb_input_buffer[usb_input_bufptr] = 0;
			usb_input_bufptr = 0;
			return USB_NO_MSG;  /* use usbFunctionRead() to obtain data */
		}
		else if(rq->bRequest == USBRQ_HID_SET_REPORT){
			/// Initialize variables for G-code parsing
			usb_input_bufptr = 0;
			return USB_NO_MSG;  /* use usbFunctionWrite() to receive data from host */
		}
	} else {
		/* ignore vendor type requests, we don't use any */
	}
	return 0;
}
/*}}}*/
// TODO rename buffer to queue
#define CMDBUF_NOT_EMPTY  (cmdbufS != cmdbufE)
#define CMDBUF_NOT_FULL   (((cmdbufE+1)%MAX_CMD_COUNT) != cmdbufS)
#define CMD_IS_NOT_BUFFERED_TYPE (usb_input_buffer[0] & 0xf0) 

void execute_command_immediate()
{
}

void execute_command_from_buffer()
{
	cli(); 				// atomic operation
				//target_nanopos[2] += 10000; nanospeed[2] = 100; // XXX
	if (cmdbuf[MAX_CMD_LENGTH*cmdbufS] == CMD_MOVE) {		/// "Move" command
		target_nanopos[0] = *((uint32_t*)(cmdbuf+(MAX_CMD_LENGTH*cmdbufS)+1));
		target_nanopos[1] = *((uint32_t*)(cmdbuf+(MAX_CMD_LENGTH*cmdbufS)+5));
		target_nanopos[2] = *((uint32_t*)(cmdbuf+(MAX_CMD_LENGTH*cmdbufS)+9));
		nanospeed[0]      = *((uint32_t*)(cmdbuf+(MAX_CMD_LENGTH*cmdbufS)+13));
		nanospeed[1]      = *((uint32_t*)(cmdbuf+(MAX_CMD_LENGTH*cmdbufS)+17));
		nanospeed[2]      = *((uint32_t*)(cmdbuf+(MAX_CMD_LENGTH*cmdbufS)+21));
	}
	
	//if ((usb_input_buffer[0] == CMD_SET_DRILL_PWM)) {	 /// Sets average voltage on the drill 
		//drill_pwm = *((uint8_t*)(usb_input_buffer+1));
	//}
	// TODO clear buf at cmdbufS
	cmdbufS = (cmdbufS + 1)%MAX_CMD_COUNT;
	sei();
}


uchar   usbFunctionWrite(uchar *data, uchar len) /*{{{*/
{	
	/// Store all chunks in temporary usb_input_buffer
	uint8_t dataptr;
	for (dataptr=0; dataptr < len; dataptr++) {
		(usb_input_buffer[usb_input_bufptr]) = (uint8_t)data[dataptr];
		usb_input_bufptr++; }
	
	/// At the last chunk, process the received message 
	if (len < 8) 
	{
		if CMD_IS_NOT_BUFFERED_TYPE {
			execute_command_immediate();}
		else
		{
			if CMDBUF_NOT_FULL 
			{ 
				cli();					// atomic (uninterrupted) copying
				for (dataptr=0; dataptr < MAX_CMD_LENGTH; dataptr++) 
				{ 
					cmdbuf[MAX_CMD_LENGTH*cmdbufE + dataptr] = usb_input_buffer[dataptr]; 
					usb_input_buffer[dataptr] = 0;// clean usb input buffer 
				}
				cmdbufE = (cmdbufE + 1)%MAX_CMD_COUNT; 	// shift the cyclic pointer
				sei();
			}
			// else drop command and  TODO report error
		}
		return 1;   /* return 1 if this was the last chunk */
	}
	return 0;
}
/*}}}*/


#define ALL_IDLE ((status[0] == STATUS_IDLE) && (status[1] == STATUS_IDLE) && (status[2] == STATUS_IDLE)) 

uchar   usbFunctionRead(uchar *data, uchar len)
{
	// return one byte containing running/idle status, end switch detection, buffer empty, buffer full 
	data[0] = 0x00;
	if ALL_IDLE       {data[0] |= 0x01;}
	if AT_END_SENSOR1 {data[0] |= 0x02;}
	if AT_END_SENSOR2 {data[0] |= 0x04;}
	if AT_END_SENSOR3 {data[0] |= 0x08;}
	return 1; 
}
/*}}}*/

int main(void) /*{{{*/
{
    
    // Enable input and output pins
    SETUP_END_SENSORS;
    MOTOR1_DDR |= MOTOR1_PINS;
    MOTOR2_DDR |= MOTOR2_PINS;
    MOTOR3_DDR |= MOTOR3_PINS;
    DRILL_PWM_DDR |= DRILL_PWM_PIN;

    DDRC |= _BV(PC4); // I2C communication pins
    DDRC |= _BV(PC5);
    
    cmdbufS = 0; cmdbufE = 0;

	/// USB initialisation
	odDebugInit(); usbInit(); usbDeviceDisconnect();  					// enforce re-enumeration
	unsigned char i; i = 0; while(--i){wdt_reset(); _delay_ms(5); }		// fake USB disconnect for > 250 ms
	usbDeviceConnect(); sei();


	/// Set up the internal interrupt timer
	//	Note: prescaler settings from datasheet: (CS02, ., CS00) = ... 
	//	(0, 0, 1) --> clkI/1; (0, 1, 0) --> clkI/8; (0, 1, 1) --> clkI/64; (1, 0, 0) --> clkI/256; (1, 0, 1) --> clkI/1024; 
	//TCCR0 |= 1<<CS02 | 1<<CS00;
	TCCR0 |=  1<<CS01;
	TIMSK |= (1 << TOIE0);
	global_pwm_counter = 0;


	/// Hardware testing: Initialize motors and drill 
	for (uint8_t m=0; m<3; m++) {
		nanospeed[m] = 1; // todo to be set by EEPROM
		nanopos[m] = ZERO_POSITION; 
		target_nanopos[m] = ZERO_POSITION; 
	}
	drill_pwm = 0;

	for(;;){				
		
		/// Check USB in any idle moment
		usbPoll();

		/// Timer interrupt is called at 7808 Hz and sets the timer_interrupt_occured variable
		if (timer_interrupt_occured == 1) {
			/// PWM check
			global_pwm_counter++; global_pwm_counter %= MICROSTEP_PER_STEP;
			/// Let the USB driver check for incoming packets (at the end of this interrupt routine) <--- todo: why?
			// PWM cycle, called at 488 Hz
			if (global_pwm_counter == 0) {
				sei(); // FIXME why??
				
				// possibly load a new command
				if (ALL_IDLE  && CMDBUF_NOT_EMPTY) execute_command_from_buffer(); // XXX test

				// Check arrival at the end sensor; if so, simply clip the motion to current position
				if AT_END_SENSOR1 { target_nanopos[1-1] = nanopos[1-1]; } 
				if AT_END_SENSOR2 { target_nanopos[2-1] = nanopos[2-1]; } 
				if AT_END_SENSOR3 { target_nanopos[3-1] = nanopos[3-1]; } 

				for (uint8_t m=0; m<3; m++) {		// repeat for each motor m 
					// Set new value of position or PWM for microstepping
					if (nanopos[m] < target_nanopos[m]) {
						if ((nanopos[m]+nanospeed[m]) <= target_nanopos[m]) 
							{nanopos[m] = nanopos[m]+nanospeed[m];}
						else 
							{nanopos[m] = target_nanopos[m];};
					};
					if (nanopos[m] > target_nanopos[m]) {
						if ((nanopos[m]-nanospeed[m]) >= target_nanopos[m]) 
							{nanopos[m] = nanopos[m]-nanospeed[m];}
						else 
							{nanopos[m] = target_nanopos[m];};	  // arrived
					};

					if (nanopos[m] == target_nanopos[m]) { status[m] = STATUS_IDLE;} 
					else { status[m] = STATUS_RUNNING; }
					// Precalculate values for the PWM routine
					real_step[m] = (nanopos[m])/NANOSTEP_PER_STEP;
					real_pwm[m] = (nanopos[m]%NANOSTEP_PER_STEP)/NANOSTEP_PER_MICROSTEP;
				}
				if (drill_pwm > 0) DRILL_PWM_PORT |= DRILL_PWM_PIN; 
			};
			// Microstepping using the lookup table
			if (global_pwm_counter == drill_pwm) DRILL_PWM_PORT &= ~(DRILL_PWM_PIN); 
			set_motor1_step(real_step[0]+((lookup_table[real_pwm[0]]>>(global_pwm_counter))&0x0001)); 
			set_motor2_step(real_step[1]+((lookup_table[real_pwm[1]]>>(global_pwm_counter))&0x0001)); 
			set_motor3_step(real_step[2]+((lookup_table[real_pwm[2]]>>(global_pwm_counter))&0x0001)); 
			timer_interrupt_occured = 0;
		};
	};
	return 0;
}
/*}}}*/

