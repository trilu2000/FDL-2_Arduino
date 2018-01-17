/*- -----------------------------------------------------------------------------------------------------------------------
*  FDL-2 arduino implementation
*  2018-01-17 <trilu@gmx.de> Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
* - -----------------------------------------------------------------------------------------------------------------------
* - all supporting functions, like pin setup, pin change interrupt handling, etc ------------------------------------------
*   special thanks to Jesse Kovarovics http://www.projectfdl.com to make this happen
* - -----------------------------------------------------------------------------------------------------------------------
*/

#ifndef _MYFUNC_h
#define _MYFUNC_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif


#include <stdint.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/atomic.h>


class waittimer {

private:	//---------------------------------------------------------------------------------------------------------
	uint32_t startTime;
	uint32_t checkTime;

public:		//---------------------------------------------------------------------------------------------------------
	waittimer();
	uint8_t  done(void);
	void     set(uint32_t wait_millis);
	uint32_t remain(void);
	uint8_t  completed(void);
};


//- pin definition ----------------------------------------------------------------------------------------------------------
#define pc_interrupt_vectors 3																// amount of pin change interrupt vectors

#define pinD0 (0)		// &DDRD, &PORTD, &PIND, 16, PCINT16, &PCICR, &PCMSK2, PCIE2, 2
#define pinD1 (1)
#define pinD2 (2)
#define pinD3 (3)
#define pinD4 (4)
#define pinD5 (5)
#define pinD6 (6)
#define pinD7 (7)

#define pinB0 (8)		// &DDRB, &PORTB, &PINB,  0,  PCINT0, &PCICR, &PCMSK0, PCIE0, 0
#define pinB1 (9)
#define pinB2 (10)
#define pinB3 (11)
#define pinB4 (12)
#define pinB5 (13)

#define pinC0 (14)
#define pinC1 (15)
#define pinC2 (16)
#define pinC3 (17)
#define pinC4 (18) 
#define pinC5 (19)
#define pinC6 (20)
#define pinC7 (21)


//- -------------------------------------------------------------------------------------------------------------------------


/*-- pin functions --------------------------------------------------------------------------------------------------------
* all pins defined as a struct, holding all information regarding port, pin, ddr, etc.
* as we support different arduino hw i tried to make it as flexible as possible. everything is defined in seperate
* hw specific files. the struct and pin manipulation function is defined in HAL_atmega.h because it is similar for all
* ATMEL hardware, the pin structs are defined in HAL_atmega_<model> while different for each cpu type. here we reference
* only on the functions defined in HAL_<type>_<model>.
*/
void set_pin_output(uint8_t pin_def);
void set_pin_input(uint8_t pin_def);
void set_pin_high(uint8_t pin_def);
void set_pin_low(uint8_t pin_def);
uint8_t get_pin_status(uint8_t pin_def);
//- -----------------------------------------------------------------------------------------------------------------------


/*-- interrupt functions --------------------------------------------------------------------------------------------------
* interrupts again are very hardware supplier related, therefor we define her some external functions which needs to be
* defined in the hardware specific HAL file. for ATMEL it is defined in HAL_atmega.h.
* you can also use the arduino standard timer for a specific hardware by interlinking the function call to getmillis()
*/
#define DEBOUNCE  5																			// debounce time for periodic check if an interrupt was raised
extern void(*pci_ptr)(uint8_t vec, uint8_t pin, uint8_t flag);
void register_PCINT(uint8_t pin_def);
uint8_t check_PCINT(uint8_t pin_def, uint8_t debounce);
void maintain_PCINT(uint8_t vec);
//- -----------------------------------------------------------------------------------------------------------------------


/*-- timer functions ------------------------------------------------------------------------------------------------------
* as i need timer0 interrupt for the encoder service i have to define an own millis() timer here.
*/
// https://github.com/zkemble/millis/blob/master/millis/
extern volatile uint32_t milliseconds;
void init_millis_timer0();																	// initialize timer0
uint32_t get_millis(void);																	// get the current time in millis


/*-- eeprom functions -----------------------------------------------------------------------------------------------------
* eeprom is very hardware supplier related, therefor we define her some external functions which needs to be defined
* in the hardware specific HAL file. for ATMEL it is defined in HAL_atmega.h.
*/
void init_eeprom(void);
void get_eeprom(uint16_t addr, uint8_t len, void *ptr);
void set_eeprom(uint16_t addr, uint8_t len, void *ptr);
void clear_eeprom(uint16_t addr, uint16_t len);


/*-- serial print functions -----------------------------------------------------------------------------------------------
* template and some functions for debugging over serial interface
* based on arduino serial class, so should work with all hardware served in arduino
* http://aeroquad.googlecode.com/svn/branches/pyjamasam/WIFIReceiver/Streaming.h
*/

class NullSerial : public Print {
public:
	virtual size_t write(uint8_t) { return (1); }
	void begin(int16_t) {}
};

NullSerial static Noserial;

#ifdef DEBUG
	#define dbg Serial
#else
	#define dbg Noserial
#endif

template<class T> Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }

const char num2char[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',  'A', 'B', 'C', 'D', 'E', 'F', };
struct _HEX {
	uint8_t *val;
	uint8_t len;
	_HEX(uint8_t v) : val(&v), len(1) {}
	_HEX(uint8_t *v, uint8_t l = 1) : val(v), len(l) {}
};
inline Print &operator <<(Print &obj, const _HEX &arg) {
	for (uint8_t i = 0; i<arg.len; i++) {
		if (i) obj.print(' ');
		obj.print(num2char[arg.val[i] >> 4]);
		obj.print(num2char[arg.val[i] & 0xF]);
	}
	return obj;
}

enum _eTIME { _TIME };
inline Print &operator <<(Print &obj, _eTIME arg) { obj.print('('); obj.print(get_millis()); obj.print(')'); return obj; }
//- -----------------------------------------------------------------------------------------------------------------------


#endif
