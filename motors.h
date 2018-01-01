#ifndef _MOTORS_h
#define _MOTORS_h

#include <Servo.h>
#include "myfunc.h"



/**
* @brief contructor for the pusher class to drive the pusher motor of a FDL-2
*
* @parameter (uint8_t) IN1 pin, (uint8_t) IN2 pin, (uint8_t) PWM pin, (uint8_t) STBY pin, (uint8_t) FRNT_SENS pin, (uint8_t) BACK_SENS pin
*/
class PusherClass {
public:
	PusherClass(uint8_t IN1, uint8_t IN2, uint8_t PWM, uint8_t STBY, uint8_t FRNT_SENS, uint8_t BACK_SENS, uint8_t &launcher_enable);

	void set_speed(uint8_t speed);	// set speed and remembers it
	void start();					// start the pusher 
	void stop();					// init the stop process

	void poll();					// poll function to operate the pusher
	uint8_t mode = 1;				// how many darts to be launched by one start

private:
	uint8_t *enable;				// pointer to an enable variable - 1 means enabled

	uint8_t pin_in1;				// remember the pin definition/numbers
	uint8_t pin_in2;
	uint8_t pin_stb;
	uint8_t pin_pwm;

	uint8_t pin_frnt;
	uint8_t pin_back;

	uint8_t operate = 3;			// 0 inactive, 1 start, 2 running, 3 returning, 4 break, 5 stopping
	uint8_t position;				// 0 unknown, 1 front sensor, 11 still front sensor, 2 after frontsensor, 12 still after frontsensor, 
									// 3 back sensor, 13 still back sensor, 4 after back sensor, 14 still after back sensor
	uint8_t pwm_store;				// remember the set speed
	uint8_t round = 0;				// pushed darts counter
};





class LauncherClass {

public:
	LauncherClass(uint8_t ESC, uint16_t min_speed, uint16_t max_speed);

	uint8_t ready;					// readiness of launcher 
	uint8_t fire_speed;				// fire speed in % of max_speed
	uint16_t speedup_time;			// holds the time the motor needs to speedup
	uint8_t standby_speed;			// standby speed in % of max_speed
	uint16_t standby_time;			// standby time in ms


	void init();					// init the launcher, write start value into the ESC
	void start();					// init the start process of the launcher 
	void stop();					// init the stop process via standby speed 

	void poll();					// poll function to operate the pusher

private:
	uint8_t mode = 0;				// 0 = stopped, 10 = stopping, 1 = standby (reduced speed), 11 = going to standby speed, 2 = fire speed, 12 = accelerating to fire speed
	uint8_t pin_esc;
	Servo myServo;					// create a servo object

	uint16_t min_speed;
	uint16_t max_speed;
	uint16_t set_speed;

	waittimer timer;
};

#endif

