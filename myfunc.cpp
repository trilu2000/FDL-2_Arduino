#include "myfunc.h"

/**
* @brief Constructor to initialize waittimer
*/
waittimer::waittimer() {
}

/**
* @brief Query if the timer has expired
*
* @return 0 if timer is still running, 1 if not.
*         If the timer was never set(), return value is 1
*/
uint8_t  waittimer::done(void) {
	if (!checkTime) return 1;																// not armed, so nothing to do
	if ((millis() - startTime) < checkTime) return 0;										// not ready yet
	checkTime = 0;																			// if we are here, timeout was happened
	return 1;																				// return a 1 for done
}

/**
* @brief Start the timer
*
* @param ms Time until timer is done() (unit: ms)
*/
void     waittimer::set(uint32_t wait_millis) {
	uint8_t armed = (wait_millis) ? 1 : 0;
	if (!armed) return;
	startTime = millis();
	checkTime = wait_millis;
}

/**
* @brief Query the remaing time until the timer is done
*
* @return Time until timer is done() (unit: ms)
*/
uint32_t waittimer::remain(void) {
	if (!checkTime) return 0;
	return (checkTime - (millis() - startTime));
}

/* returns the status of the timer
* 0 not active
* 1 active and remaining time is 0 or below, but not progressed via done()
* 2 active and remaining time is above 0 */
uint8_t waittimer::completed(void) {
	if (!checkTime) return 0;																// not armed, so return not active
	else if ((millis() - startTime) >= checkTime) return 1;									// timer done, but not progressed
	else return 2;																			// time not ready, need some additional time
}






/*-- pin functions --------------------------------------------------------------------------------------------------------
* concept of pin functions is a central definition of pin and interrupt registers as a struct per pin. handover of pin
* information is done by forwarding a pointer to the specific function and within the function all hardware related
* setup and switching is done.
*/

/* set a specific pin as output */
void set_pin_output(uint8_t pin_def) {
	uint8_t bit = digitalPinToBitMask(pin_def);
	uint8_t port = digitalPinToPort(pin_def);
	volatile uint8_t *reg;

	reg = portModeRegister(port);
	*reg |= bit;
}
/* set the pin as input */
void set_pin_input(uint8_t pin_def) {
	uint8_t bit = digitalPinToBitMask(pin_def);
	uint8_t port = digitalPinToPort(pin_def);
	volatile uint8_t *reg;

	reg = portModeRegister(port);
	*reg &= ~bit;
}

/* set high level on specific pin */
void set_pin_high(uint8_t pin_def) {
	uint8_t bit = digitalPinToBitMask(pin_def);
	uint8_t port = digitalPinToPort(pin_def);

	volatile uint8_t *out;
	out = portOutputRegister(port);
	*out |= bit;
}
/* set a low level on a specific pin */
void set_pin_low(uint8_t pin_def) {
	uint8_t bit = digitalPinToBitMask(pin_def);
	uint8_t port = digitalPinToPort(pin_def);

	volatile uint8_t *out;
	out = portOutputRegister(port);
	*out &= ~bit;
}
/* detect a pin input if it is high or low */
uint8_t get_pin_status(uint8_t pin_def) {
	uint8_t bit = digitalPinToBitMask(pin_def);
	uint8_t port = digitalPinToPort(pin_def);

	if (*portInputRegister(port) & bit) return HIGH;
	return LOW;
}
//- -----------------------------------------------------------------------------------------------------------------------




/*-- interrupt functions --------------------------------------------------------------------------------------------------
* based on the same concept as the pin functions. everything pin related is defined in a pin struct, handover of the pin
* is done by forwarding a pointer to the struct. pin definition is done in HAL_<cpu>.h, functions are declared in HAL_<vendor>.h
*/
struct  s_pcint_vector {
	volatile uint8_t *PINREG;
	uint8_t curr;
	uint8_t prev;
	uint8_t mask;
	uint32_t time;
};
volatile s_pcint_vector pcint_vector[pc_interrupt_vectors];									// define a struct for pc int processing

/* function to register a pin interrupt */
void register_PCINT(uint8_t def_pin) {
	set_pin_input(def_pin);																	// set the pin as input
	set_pin_high(def_pin);																	// key is connected against ground, set it high to detect changes

																							// need to get vectore 0 - 2, depends on cpu
	uint8_t vec = digitalPinToPCICRbit(def_pin);											// needed for interrupt handling and to sort out the port
	uint8_t port = digitalPinToPort(def_pin);												// need the pin port to get further information as port register
	if (port == NOT_A_PIN) return;															// return while port was not found

	pcint_vector[vec].PINREG = portInputRegister(port);										// remember the input register
	pcint_vector[vec].mask |= digitalPinToBitMask(def_pin);									// set the pin bit in the bitmask

	*digitalPinToPCICR(def_pin) |= _BV(digitalPinToPCICRbit(def_pin));						// pci functions
	*digitalPinToPCMSK(def_pin) |= _BV(digitalPinToPCMSKbit(def_pin));						// make the pci active
	
	//pcint_vector[vec].curr |= get_pin_status(def_pin);									// remember current status of the port bit
	//pcint_vector[vec].prev = pcint_vector[vec].curr;										// and set it as previous while we check for changes
	maintain_PCINT(vec);
	pcint_vector[vec].time = millis() - DEBOUNCE;
}

/* period check if a pin interrupt had happend */
uint8_t check_PCINT(uint8_t def_pin, uint8_t debounce) {
	// need to get vectore 0 - 3, depends on cpu
	uint8_t vec = digitalPinToPCICRbit(def_pin);											// needed for interrupt handling and to sort out the port
	uint8_t bit = digitalPinToBitMask(def_pin);												// get the specific bit for the asked pin

	uint8_t status = pcint_vector[vec].curr & bit ? 1 : 0;									// evaluate the pin status
	uint8_t prev = pcint_vector[vec].prev & bit ? 1 : 0;									// evaluate the previous pin status

	if (status == prev) return status;														// check if something had changed since last time
	if (debounce && ((millis() - pcint_vector[vec].time) < DEBOUNCE)) return prev;			// seems there is a change, check if debounce is necassary and done

	pcint_vector[vec].prev ^= bit;															// if we are here, there was a change and debounce check was passed, remember for next time

	if (status) return 3;																	// pin is 1, old was 0
	else return 2;																			// pin is 0, old was 1
}

void(*pci_ptr)(uint8_t vec, uint8_t pin, uint8_t flag);

/* internal function to handle pin change interrupts */
void maintain_PCINT(uint8_t vec) {
	pcint_vector[vec].curr = *pcint_vector[vec].PINREG & pcint_vector[vec].mask;			// read the pin port and mask out only pins registered
	pcint_vector[vec].time = millis();														// store the time, if debounce is asked for

	if (pci_ptr) {
		uint8_t pin_int = pcint_vector[vec].curr ^ pcint_vector[vec].prev;					// evaluate the pin which raised the interrupt
		pci_ptr(vec, pin_int, pcint_vector[vec].curr & pin_int);							// callback the interrupt function in user sketch
	}
}

/* interrupt vectors to catch pin change interrupts */
#ifdef PCIE0
ISR(PCINT0_vect) {
	maintain_PCINT(0);
}
#endif

#ifdef PCIE1
ISR(PCINT1_vect) {
	maintain_PCINT(1);
}
#endif

#ifdef PCIE2
ISR(PCINT2_vect) {
	maintain_PCINT(2);
}
#endif

#ifdef PCIE3
ISR(PCINT3_vect) {
	maintain_PCINT(3);
}
#endif
//- -----------------------------------------------------------------------------------------------------------------------

