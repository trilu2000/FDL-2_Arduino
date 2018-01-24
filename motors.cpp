/*- -----------------------------------------------------------------------------------------------------------------------
*  FDL-2 arduino implementation
*  2018-01-17 <trilu@gmx.de> Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
* - -----------------------------------------------------------------------------------------------------------------------
* - Motor drivers, pusher and launcher-------------------------------------------------------------------------------------
*   special thanks to Jesse Kovarovics http://www.projectfdl.com to make this happen
* - -----------------------------------------------------------------------------------------------------------------------
*/

#include "motors.h"

#ifdef DEBUG_PUSHER
#define dbg_p Serial
#else
#define dbg_p Noserial
#endif

#ifdef DEBUG_LAUNCHER
#define dbg_l Serial
#else
#define dbg_l Noserial
#endif


PusherClass::PusherClass(uint8_t IN1, uint8_t IN2, uint8_t PWM, uint8_t STBY, uint8_t FRNT_SENS, uint8_t BACK_SENS, uint8_t &launcher_enable)
	: pin_in1(IN1), pin_in2(IN2), pin_stb(STBY), pin_pwm(PWM), pin_frnt(FRNT_SENS), pin_back(BACK_SENS), enable(&launcher_enable) {

	set_pin_output(pin_in1);													// the tb6612 chip has 2 inputs for driving the motor
	set_pin_low(pin_in1);														// we need to set them as output and low at the arduino side

	set_pin_output(pin_in2);
	set_pin_low(pin_in2);

	set_pin_output(pin_pwm);													// pwm drives the speed of the pusher motor
	set_speed(0);																// we use the standard arduino function for it

	set_pin_output(pin_stb);													// not sure if the standby is needed, but 
	set_pin_high(pin_stb);														// it needs to be a high level for the tb6612 chip

	register_PCINT(pin_frnt);													// the pusher has two sensors to detect the position
	register_PCINT(pin_back);													// back and front. it is needed for driving the motor

	if (get_pin_status(pin_back) == 0) position = 3;							// if the back sens is raised, we have a stable position

	pcint_callback = this;														// we register a callback to get the pc interrupts into the class
	pci_ptr = &pcint_hook;

	operate = 3;																// we start in returning mode			
}

void PusherClass::set_speed(uint8_t speed) {
	analogWrite(pin_pwm, speed);												// standard arduino function for PWM
}
void PusherClass::start() {
	operate = 1;																// we need to set the operateing mode
	round = 0;																	// reset the round counter while we are started
	dbg_p << F("P::set start ") << _TIME << '\n';								// some debug
}
void PusherClass::stop() {
	if (operate >= 3) return;													// we are already in stopping mode
	if ((*mode) && (round < *mode)) return;										// don't now, while count is in use and not complete
	operate = 3;																// enter go back mode
	dbg_p << F("P::stop needed ") << _TIME << '\n';								// some debug
}

void PusherClass::poll() {

	// 0 inactive, 1 starting, 2 running, 3 returning, 4 breaking, 5 stopping
	if (operate == 1) {							// indicates that the pusher needs to be started

		if (*enable != 1) return;												// launcher is not up to speed

		set_speed(255);															// set full speed
		set_pin_high(pin_in1);													// start the motor
		set_pin_low(pin_in2);
		operate = 2;															// and indicate that we are in operate mode
		dbg_p << F("P::started ") << _TIME << '\n';


	} else if (operate == 2) {					// pusher runs, check count mode

		if ((*mode) && (round >= *mode)) {										// check if we are in count mode and reached the target
			stop();																// slow down and start stop operation
			dbg_p << F("P::reached count: ") << round << ' ' << _TIME << '\n';	// some debug
		}


	} else if (operate == 3) {					// pusher is in return mode					
		// several actions are started within the pcint function

		if (position == 0) {					// unknown position is indicated while just booted, so we start the motor to get a postion via the position sensors

			set_speed(160);														// set a slow speed
			set_pin_high(pin_in1);												// start the motor to get a default position
			set_pin_low(pin_in2);
			set_pin_high(pin_stb);												// as we start the sketch with position = 0 and operate = 3 (return mode)
			position = 10;														// set position to unknown but with motor started
			dbg_p << F("P::no position, get one ") << _TIME << '\n';			// some debug

		} else if (position == 2) {				// we left the front sensor, the short break is done within the pcint function 

			if (!timer.done()) return;											// pcint has a timer set to slowdown the motor
			set_speed(80);														// set a slow speed
			set_pin_high(pin_in1);												// start the motor again
			set_pin_low(pin_in2);
			position = 12;														// set a new position status
			dbg_p << F("P::break done, motor on slow speed ") << _TIME << '\n';	// some debug

		}


	} else if (operate == 4) {					// pusher is in breaking mode
		// wait for finishing the stopping time and check if we are at the right position 
		// if we are too far, we see pos 4 in the pcint function and can return the motor there

		if (!timer.done()) return;

		set_pin_low(pin_in1);													// release motor 
		set_pin_low(pin_in2);

		if (position == 3) {					// we are at the right position 
			operate = 5;														// indicate finish
			dbg_p << F("P::stopped at the right position ") << _TIME << '\n';	// some debug
		}


	} else if (operate == 5) {					// pusher is stopped
		dbg_p << F("P::finish stop, wait for action ") << _TIME << '\n';
		operate = 0;															// set operating mode to inactive
		round = 0;																// and reset the round counter
	}

	if (position_old == position) return;										// only if the position had changed to the last loop								
	position_old = position;													// to identify a change

	if (position == 1) dbg_p << F("P::dart ") << round << ' ' << _TIME << '\n';	// some debug

}

void PusherClass::callback(uint8_t vec, uint8_t pin, uint8_t flag) {
	/* this is the pc interrupt function to detect the status of the front and back sensor
	** one turn of the pusher disc has several stati, here we are handling 1 front, 2 after front, 3 back and 4 after back sensor
	** on status 1 we count the pushed darts 
	** on status 2 we start to slow down the motor if stop is required
	** on status 3 we stop the motor if stop is required
	** on status 4 we returning the motor if a stop is required because we have overrun the stop at the back sensor	*/

	uint32_t cur_millis = get_millis();											// remember the time for debouncing

	/* back sensor handling */
	static uint32_t last_back_millis;											// variable to remember static on the time of the last event
	if ((digitalPinToPCICRbit(pin_back) == vec) && (digitalPinToBitMask(pin_back) == pin)) {// we only want to see the interrupt for the specific pin

		if (cur_millis - last_back_millis < 5) return;							// last event was <5 ms ago, seems to be a bounce
		last_back_millis = cur_millis;											// remember the time for debounce

		uint8_t stat = flag & digitalPinToBitMask(pin_back);					// filter if the button was pushed (0) or released (64)
		//dbg << "bb: " << _HEX(stat) << ' ' << ((stat) ? 'r' : 'p') << ' ' << _TIME << '\n';

		if (!stat) {															// we are at the back sensor 
			position = 3;														// set the position flag
			if (operate == 3) {													// if we are in returning mode, we should have a slow motor and reached now the end position
				operate = 4;													// set the new operate mode - breaking
				set_pin_high(pin_in1);											// set breaking mode on motor
				set_pin_high(pin_in2);
				timer.set(200);													// and some time for slowing down, follow up is in the poll function
			}

		} else {																// back sensor left
			position = 4;														// remember the position
			if (operate == 4) {													// seems we are slipped over the back sensor, so we return the motor
				operate = 3;													// we are in returning mode
				set_speed(100);													// set a slow speed
				set_pin_low(pin_in1);											// start the motor in the oposite direction again 
				set_pin_high(pin_in2);
			}
		}
	}


	/* front sensor handling */
	static uint32_t last_frnt_millis;											// remember static on the time of the last event
	if ((digitalPinToPCICRbit(pin_frnt) == vec) && (digitalPinToBitMask(pin_frnt) == pin)) {// we only want to see the interrupt for the specific pin

		if (cur_millis - last_frnt_millis < 5) return;							// last event was <5 ms ago, seems to be a bounce
		last_frnt_millis = cur_millis;											// remember the time for debounce

		uint8_t stat = flag & digitalPinToBitMask(pin_frnt);					// filter if the button was pushed (0) or released (128)
		//dbg_p << "fb: " << _HEX(stat) << ' ' << ((stat) ? 'r' : 'p') << ' ' << _TIME << '\n';

		if (!stat) {															// we are at the front sensor 
			position = 1;														// remember the position
			round++;															// increase the round counter (darts pushed)

		} else {																// front sensor left
			position = 2;														// set the new position
			if (operate == 3) {													// if we are in returning mode, we are breaking the motor for some time and start it slower to return to the next position
				set_pin_high(pin_in1);											// set breaking mode of tb6612 chip
				set_pin_high(pin_in2);
				timer.set(100);													// set some time, follow up is in main loop
			}
		}
	}

}


void pcint_hook(uint8_t vec, uint8_t pin, uint8_t flag) {						// linked to the pin change interrupt, not debounced
	pcint_callback->callback(vec, pin, flag);									// call the hook function
}


LauncherClass::LauncherClass(uint8_t ESC, uint16_t min_speed, uint16_t max_speed) : pin_esc(ESC), min_speed(min_speed), max_speed(max_speed) {
	ready = 0;
}

void LauncherClass::init() {
	myServo.attach(pin_esc, min_speed, max_speed);								// attaches the servo pin to the servo object
	myServo.writeMicroseconds(0);												// and set it off

	dbg_l << F("L::init, min_speed: ") << min_speed << F(", max_speed: ") << max_speed << ' ' << _TIME << '\n';
}
void LauncherClass::start() {

	set_speed = max_speed / 100 * *fire_speed;									// calculate the needed speed - max_speed is an absolute value and fire_speed a percentage value

	/* state machine modes: 0 = stopped, 10 = stopping, 1 = standby (reduced speed), 
	** 11 = going to standby speed, 2 = fire speed, 12 = accelerating to fire speed */
	uint16_t set_timer;															// generate a variable to store the speedup time against different circumsdances
	if (mode == 0) set_timer = *speedup_time;									// coming from stopped, full time needed
	else if (mode == 2) set_timer = 0;											// we are already in fire mode, no additional time needed
	else set_timer = *speedup_time / 2;											// standby, decrease to standby or accelerating to fire, not the fulltime needed

	mode = 12;																	// set state machine to 'accelerating to fire speed'
	timer.set(set_timer);														// set the timer accordingly
	myServo.writeMicroseconds(set_speed);										// and write the new speed into the esc

	dbg_l << F("L::set start, speed: ") << *fire_speed << F(", set_speed: ") << set_speed << F(", speedup_time: ") << *speedup_time << F(", set_timer: ") << set_timer << ' ' << _TIME << '\n';
}
void LauncherClass::stop() {
	/* stop means, we are reducing the speed of the launcher to a standby level for a certain time
	** here we are setting a new status of the state machine */

	if (set_speed) set_speed = max_speed / 100 * *standby_speed;				// calculate the standby speed 				

	/* state machine modes: 0 = stopped, 10 = stopping, 1 = standby (reduced speed),
	** 11 = going to standby speed, 2 = fire speed, 12 = accelerating to fire speed */
	uint16_t set_timer = *speedup_time / 2;										// we need some time to slow down

	ready = 0;																	// indicate the pusher that he cannot fire
	mode = 11;																	// we are going to standby speed
	timer.set(set_timer);														// set the timer accordingly
	myServo.writeMicroseconds(set_speed);										// write the new speed into the esc

	dbg_l << F("L::set stop, speed: ") << *standby_speed << F(", set_speed: ") << set_speed << F(", set_timer: ") << set_timer << ' ' << _TIME << '\n';
}

void LauncherClass::poll() {
	/* state machine modes: 0 = stopped, 10 = stopping, 1 = standby (reduced speed),
	** 11 = going to standby speed, 2 = fire speed, 12 = accelerating to fire speed */

	if (!timer.done()) return;													// waiting for a finished timer, leave


	if (mode == 12) {				// accelerating to fire														
		/* triggered in the start function, if we are here the start time has finished already */
		ready = 1;																// signalize the pusher that he is allowed to fire
		mode = 2;																//set status to 'fire speed'
		dbg_l << F("L::accelerate done ") << _TIME << '\n';


	} else if (mode == 11) {		// reducing speed to standby mode
		/* triggered in the stop function, if we are here the stop time has finsished */
		timer.set(*standby_time);												// set the standby timer
		mode = 1;																// set status 'standby' - we are on reduced speed
		dbg_l << F("L::standby for ") << *standby_time  << F("ms ") << _TIME << '\n';


	} else if (mode == 1) {			// standby time is over
		/* triggered by the state machine itself, standby is over, we need to stop the motor */
		uint16_t set_timer = *speedup_time / 2;									// calculate the time for the stop process
		timer.set(set_timer);													// set the timer accordingly
		myServo.writeMicroseconds(0);											// set the esc to stop
		mode = 10;																// set the status to stopping mode
		dbg_l << F("L::stopping for ") << set_timer << F("ms ") << _TIME << '\n';


	} else if (mode == 10) {		// stopping mode
		/* triggered by the state machine itself, stopping time is over, we have a new status */
		mode = 0;																// we are stopped
		dbg_l << F("L::stopped!") << _TIME << '\n';

	}
}