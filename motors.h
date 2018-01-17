/*- -----------------------------------------------------------------------------------------------------------------------
*  FDL-2 arduino implementation
*  2018-01-17 <trilu@gmx.de> Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
* - -----------------------------------------------------------------------------------------------------------------------
* - Motor drivers, pusher and launcher-------------------------------------------------------------------------------------
*   special thanks to Jesse Kovarovics http://www.projectfdl.com to make this happen
* - -----------------------------------------------------------------------------------------------------------------------
*/

#ifndef _MOTORS_h
#define _MOTORS_h

#include <Servo.h>
#include "myfunc.h"

//#define DEBUG_PUSHER
//#define DEBUG_LAUNCHER



/**
* @brief contructor for the pusher class to drive the pusher motor of a FDL-2
*
* @parameter (uint8_t) IN1 pin, (uint8_t) IN2 pin, (uint8_t) PWM pin, (uint8_t) STBY pin, (uint8_t) FRNT_SENS pin, (uint8_t) BACK_SENS pin
*/
class PusherClass {
public:
	uint8_t mode;					// how many darts to be launched by one start
	PusherClass(uint8_t IN1, uint8_t IN2, uint8_t PWM, uint8_t STBY, uint8_t FRNT_SENS, uint8_t BACK_SENS, uint8_t &launcher_enable);

	void set_speed(uint8_t speed);	// set speed and remembers it
	void start();					// start the pusher 
	void stop();					// init the stop process

	void poll();					// poll function to operate the pusher

	void callback(uint8_t vec, uint8_t pin, uint8_t flag);

private:
	uint8_t *enable;				// pointer to an enable variable - 1 means enabled (pusher shall start only while the launcher is at full speed)

	uint8_t pin_in1;				// remember the pin definition/numbers
	uint8_t pin_in2;
	uint8_t pin_stb;
	uint8_t pin_pwm;

	uint8_t pin_frnt;				// remember the front and back sensor pins
	uint8_t pin_back;

	uint8_t operate;				// state machine, 0 inactive, 1 starting, 2 running, 3 returning, 4 breaking, 5 stopping
	uint8_t position;				// 0 unknown, 10 unknown but motor started, 1 front sensor, 2 after frontsensor, 12 after frontsensor but slow speed, 3 back sensor, 4 after back sensor 
	uint8_t position_old;			// to detect changes of the position
	uint8_t round;					// pushed darts counter

	waittimer timer;				// mainly used for breaking the motor as non block delay in the poll function
};

static PusherClass *pcint_callback;
void pcint_hook(uint8_t vec, uint8_t pin, uint8_t flag);




/**
* @brief contructor for the launcher class to accelerate the darts in a FDL-2
*
* @parameter (uint8_t) ESC pin, (uint16_t) min_speed, (uint16_t) max_speed
*/
class LauncherClass {
public:
	uint8_t ready;					// signals readiness of launcher 
	uint8_t fire_speed;				// fire speed in % of max_speed
	uint16_t speedup_time;			// holds the time the motor needs to speedup
	uint8_t standby_speed;			// standby speed in % of max_speed
	uint16_t standby_time;			// standby time in ms

	LauncherClass(uint8_t ESC, uint16_t min_speed, uint16_t max_speed);

	void init();					// init the launcher, write start value into the ESC
	void start();					// init the start process of the launcher 
	void stop();					// init the stop process via standby speed 

	void poll();					// poll function to operate the pusher

private:
	uint8_t mode = 0;				// 0 = stopped, 10 = stopping, 1 = standby (reduced speed), 11 = going to standby speed, 2 = fire speed, 12 / 22 = accelerating to fire speed
	uint8_t pin_esc;
	Servo myServo;					// create a servo object

	uint16_t min_speed;
	uint16_t max_speed;
	uint16_t set_speed;

	waittimer timer;
};

#endif

