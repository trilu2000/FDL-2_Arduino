#include "motors.h"

PusherClass::PusherClass(uint8_t IN1, uint8_t IN2, uint8_t PWM, uint8_t STBY, uint8_t FRNT_SENS, uint8_t BACK_SENS, uint8_t &launcher_enable)
	: pin_in1(IN1), pin_in2(IN2), pin_stb(STBY), pin_pwm(PWM), pin_frnt(FRNT_SENS), pin_back(BACK_SENS), enable(&launcher_enable) {

	set_pin_output(pin_in1);
	set_pin_low(pin_in1);

	set_pin_output(pin_in2);
	set_pin_low(pin_in2);

	set_pin_output(pin_pwm);
	set_speed(0);

	set_pin_output(pin_stb);
	set_pin_high(pin_stb);

	register_PCINT(pin_frnt);
	register_PCINT(pin_back);
}



void PusherClass::poll() {
	/* understand the position of the pusher motor is crutial to drive it */

	uint8_t position_old = position;

	uint8_t back_sens_temp = check_PCINT(pin_back, 1);
	uint8_t frnt_sens_temp = check_PCINT(pin_frnt, 1);

	// 0 unknown, 1 front sensor, 11 still front sensor, 2 after frontsensor, 12 still after frontsensor, 
	// 3 back sensor, 13 still back sensor, 4 after back sensor, 14 still after back sensor
	if (frnt_sens_temp == 2) position = 1;										// front sensor newly raised
	else if (frnt_sens_temp == 0) position = 11;								// still on the front sensor
	else if (back_sens_temp == 2) position = 3;									// back sensor newly raised
	else if (back_sens_temp == 0) position = 13;								// still on the back sensor
	else {
		if ((position == 1) || (position == 11)) position = 2;					// we are coming from the frontsensor, but it is not active anymore
		else if (position == 2)                      position = 12;				// last time it was first time after the front sensor, now we are still after front sensor
		else if ((position == 3) || (position == 13)) position = 4;				// we are coming from the back sensor, now we are first time after back sensor
		else if (position == 4)                      position = 14;				// last time it was first time after back sensor, now we are still after back sensor
	}

	if (position_old != position) {
		//dbg << frnt_sens_temp << ',' << back_sens_temp << ", mod: " << operate << ", pwm: " << pwm_store << ", pos: " << position << '\n';
		if (position == 1) {													// and foam dart was pushed to the motor
			round++;
			dbg << F("P::dart:") << round << '\n';								// some debug
		}

	}


	// 0 inactive, 1 start, 2 running, 3 returning, 4 break, 5 stopping
	if (operate == 1) {							// indicates that the pusher needs to be started
		if (*enable != 1) return;												// launcher is not up to speed
		dbg << F("P::starting...\n");
		set_speed(255);															// set full speed
		set_pin_high(pin_in1);													// start the motor
		set_pin_low(pin_in2);
		operate = 2;															// and indicate that we are in operate mode

	} else if (operate == 2) {					// pusher runs, check count mode
		if ((mode) && (round >= mode)) {										// check if we are in count mode and reached the target
			dbg << F("P::reached count...\n");										// some debug
			stop();																// slow down and start stop operation
		}

	} else if (operate == 3) {					// pusher is in return mode					
		if (position == 0) {													// while started new, no default position
			dbg << F("P::no position, get one\n");								// some debug
			set_speed(120);														// set a slow speed
			set_pin_high(pin_in1);												// start the motor to get a default position
			set_pin_low(pin_in2);
			position = 10;														// change the status

		} else if ((position == 3) || (position == 13)) {
			dbg << F("P::end position reached\n");								// some debug
			if (pwm_store > 100) set_speed(80);									// to fast, do an extra round and slow down
			else operate = 4;													// break and stop

		}


	} else if (operate == 4) {					// pusher is in breaking mode
		dbg << F("P::break mode\n");											// some debug

		set_pin_high(pin_in1);													// init the break 
		set_pin_high(pin_in2);
		delay(100);																// wait for some time

		if ((position == 3) || (position == 13)) {								// check if we are still at the end 
			dbg << F("stopped at the right position\n");
			operate = 5;														// release the brake and wait in the stop position

		} else {																// we slipped over
			dbg << F("too far, go back\n");
			set_speed(80);														// set a slow speed
			set_pin_low(pin_in1);												// start the motor in the oposite direction again 
			set_pin_high(pin_in2);
		}

	} else if (operate == 5) {					// pusher is stopped
		dbg << F("P::finish stop, wait for action\n");
		set_pin_low(pin_in1);													// release motor 
		set_pin_low(pin_in2);
		operate = 0;															// set operating mode to inactive
		round = 0;																// and reset the round counter
	}
}




void PusherClass::set_speed(uint8_t speed) {
	pwm_store = speed;
	analogWrite(pin_pwm, speed);
}

void PusherClass::start() {
	dbg << F("P::start\n");
	//count = mode;																// how many darts should be fired
	operate = 1;																// we need to set the operateing mode
	round = 0;																	// reset the round counter while we are started
}

void PusherClass::stop() {
	if (operate >= 3) return;													// we are already in stopping mode
	dbg << F("P::stop\n");

	if ((mode) && (round < mode)) return;										// don't now, while count is in use and not complete
	//dbg << F("P::stop needed\n");

	set_pin_high(pin_in1);														// break
	set_pin_high(pin_in2);
	delay(100);																	// for 100 ms

	set_speed(80);																// slow down
	set_pin_high(pin_in1);														// run again, but slow
	set_pin_low(pin_in2);

	operate = 3;																// enter go back mode
}






LauncherClass::LauncherClass(uint8_t ESC, uint16_t min_speed, uint16_t max_speed) : pin_esc(ESC), min_speed(min_speed), max_speed(max_speed) {
	ready = 0;
}

void LauncherClass::init() {
	myServo.attach(pin_esc, min_speed, max_speed);								// attaches the servo pin to the servo object
	myServo.writeMicroseconds(0);															// and set it off

	dbg << F("L::init - min_speed: ") << min_speed << F(", max_speed: ") << max_speed << ' ' << _TIME << '\n';
}

void LauncherClass::start() {
	set_speed = max_speed / 100 * fire_speed;

	uint16_t set_timer;
	if (mode == 2) set_timer = 0;												// we are already in fire mode, no additional time needed
	else if (mode == 0) set_timer = speedup_time;								// coming from stopped, full time needed
	else set_timer = speedup_time / 2;											// standby, decrease to standby or accelerating to fire, not the fulltime needed

	mode = 12;
	timer.set(set_timer);
	myServo.writeMicroseconds(set_speed);

	dbg << F("L::start   - speed: ") << fire_speed << F(", set_speed: ") << set_speed << F(", speedup_time: ") << speedup_time << F(", set_timer: ") << set_timer << ' ' << _TIME << '\n';
}

void LauncherClass::stop() {
	set_speed = max_speed / 100 * standby_speed;

	uint16_t set_timer;
	set_timer = speedup_time / 2;

	ready = 0;
	mode = 11;
	timer.set(set_timer);
//	myServo.writeMicroseconds(0);
	myServo.writeMicroseconds(set_speed);

	dbg << F("L::stop     - speed: ") << standby_speed << F(", set_speed: ") << set_speed << F(", set_timer: ") << set_timer << ' ' << _TIME << '\n';
}

void LauncherClass::poll() {
	// 0 = stopped, 10 = stopping, 1 = standby (reduced speed), 11 = going to standby speed, 2 = fire speed, 12 = accelerating to fire speed
	if (!timer.done()) return;

	if (mode == 12) {				// accelerating to fire														
		mode = 2;																// we are now ready to fire
		ready = 1;																// signalize it
		dbg << F("L::fire    - ready ") << _TIME << '\n';

	} else if (mode == 11) {		// reducing speed to standby mode
		//set_speed = max_speed / 100 * standby_speed;
		mode = 1;																// we are in standby speed
		timer.set(standby_time);												// how long do we need to stay in standby
		//myServo.writeMicroseconds(set_speed);
		dbg << F("L::standby  - time: ") << standby_time  << ' ' << _TIME << '\n';

	} else if (mode == 1) {			// standby time is over
		uint16_t set_timer;
		set_timer = speedup_time / 2;

		mode = 10;																// we are in stopping mode
		timer.set(set_timer);													// how long do we need to stay in standby
		set_speed = 0;
		myServo.writeMicroseconds(set_speed);
		dbg << F("L::stopping - time: ") << set_timer << ' ' << _TIME << '\n';

	} else if (mode == 10) {		// stopping mode
		mode = 0;																// we are stopped
		dbg << F("L::stopped  - ") << _TIME << '\n';

	}

}