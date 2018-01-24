/*- -----------------------------------------------------------------------------------------------------------------------
*  FDL-2 arduino implementation
*  2018-01-17 <trilu@gmx.de> Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
* - -----------------------------------------------------------------------------------------------------------------------
* - main sketch -----------------------------------------------------------------------------------------------------------
*   special thanks to Jesse Kovarovics http://www.projectfdl.com to make this happen
* - -----------------------------------------------------------------------------------------------------------------------
*/

#define DEBUG

/* myfunc library holds the waittimer, some hardware setup functions and the pin change interrupt handling */
#include "myfunc.h"
// ------------------------------------------------------------------------------------------------


/* display related */
// https://github.com/olikraus/u8g2/wiki/u8g2install
#include <U8g2lib.h>
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#include "mylogo.h"
waittimer display_timer;
uint8_t display_timeout;
uint8_t display_mode;
uint8_t menu_item, menu_select;
// ------------------------------------------------------------------------------------------------


/* everything encoder related */
// https://github.com/0xPIT/encoder/tree/arduino
#include <ClickEncoder.h>
ClickEncoder encoder(pinC1, pinC0, 0, 4, 0);
waittimer encoder_timeout;
#define encoder_click   pinC2
// ------------------------------------------------------------------------------------------------


/* launcher to speed up the darts and */
/* pusher to shift the dart into the launcher */
#include <Servo.h>
#include "motors.h"
LauncherClass launcher(pinB2, 1000, 2000);
uint8_t x = 1;
PusherClass pusher(pinB5, pinB4, pinB3, pinB1, pinD7, pinD6, launcher.ready);
// ------------------------------------------------------------------------------------------------


/* battery related, functionality sits within the loop function in the main sketch */
// 892 = 12.35 Volt 
#define fdl2_battery    pinC3
waittimer battery_timer;
uint8_t battery_level;
// ------------------------------------------------------------------------------------------------

struct s_settings {
	uint8_t  mode = 2;															// how many darts per fire push
	uint8_t  fire_speed = 80;													// fire speed in % of max_speed
	uint16_t speedup_time = 500;												// holds the time the motor needs to speedup
	uint8_t  standby_speed = 50;												// standby speed in % of max_speed
	uint16_t standby_time = 500;												// standby time in ms
}settings;

#define fdl2_fire       pinD5


void setup() {
	dbg.begin(115200);
	dbg << F("\n\n\nFDL-2 Arduino v0.1\n\n");									// init serial interface and some debug

	init_millis_timer0();														// init the timer0
	init_eeprom();																// init the eeprom

	u8g2.begin();																// init the display and show some start message
	display_welcome();															// show the welcome screen
	display_timer.set(5000);													// schedule next regular update

	analogReference(INTERNAL);													// battery reference to 1.1 Volt
	analogRead(fdl2_battery);
	battery_timer.set(1000);

	launcher.init();															// init the launcher
	register_PCINT(encoder_click);												// init and register the click encoder button
	register_PCINT(fdl2_fire);													// init and register the fire button as interrupt

	/* read defaults from eeprom in case that the magic number is ok */
	dbg << F("read settings from eeprom\n");

	uint16_t magic;
	get_eeprom(0, 2, &magic);
	if (magic != 0x1236) {														// magic number doesn't fit, so a new config needs written
		magic = 0x1236;															// set a magic number
		set_eeprom(0, 2, &magic);												// write the magic to the eeprom
		set_eeprom(2, 7, &settings);											// write the settings
		dbg << F("magic doesn't fit, write defaults\n");
	}

	get_eeprom(2, 7, &settings);												// read the settings

	pusher.mode = &settings.mode;												// how many darts per fire push
	launcher.fire_speed = &settings.fire_speed;									// fire speed in % of max_speed
	launcher.speedup_time = &settings.speedup_time;								// holds the time the motor needs to speedup
	launcher.standby_speed = &settings.standby_speed;							// standby speed in % of max_speed
	launcher.standby_time = &settings.standby_time;								// standby time in ms
	
	dbg << F("init complete, mode: ") << *pusher.mode << F(", speed: ") << *launcher.fire_speed << F(", speedup_time: ") << *launcher.speedup_time << F(", standby_speed: ") << *launcher.standby_speed << F(", standby_time: ") << *launcher.standby_time << F("\n\n");
}


void loop() {

	/* general poll function */
	pusher.poll();
	launcher.poll();


	/* poll the encoder regulary */
	int8_t enc_value = encoder.getValue();										// check if the encoder value had changed
	if (enc_value > 0) encoder_up(enc_value);
	if (enc_value < 0) encoder_down(enc_value);
	
	int8_t enc_button = check_PCINT(encoder_click, 1);							// check the encoder button
	if (enc_button > 1) encoder_button(enc_button);

	if (encoder_timeout.done()) {
		menu_item = 0;
		menu_select = 0;
	}


	/* poll the status display function */
	if (display_timer.done()) {
		display_status();
		display_timer.set(10000);
	}


	/* poll the battery measurement function */
	if (battery_timer.done()) {
		uint32_t bat_value = analogRead(fdl2_battery);							// read the io pin
		bat_value *= 1385;														// some math to get the milli volt value
		bat_value /= 1000;
		//dbg << F("bat_value: ") << bat_value << '\n';

		if (bat_value > 990) battery_level = (bat_value - 990) * 10 / 27;		// calculate the percentage level of the battery
		else battery_level = 0;													// this value is available outside of this if clause

		display_battery_update(battery_level);									// write it into the display
		battery_timer.set(5000);												// set next time to measure
	}


	/* check fire button continously and drive pusher */
	uint8_t x = check_PCINT(fdl2_fire, 1);
	if (x == 2) {
		pusher.start();
		launcher.start();
		dbg << F("M::Fire button pushed\n");

	} else if (x == 3) {
		pusher.stop();
		launcher.stop();
		dbg << F("M::Fire button released\n");
	}

}


void encoder_up(int8_t x) {

	if (menu_select == 0) {
		if (menu_item >= 2) menu_item = 0;
		else menu_item++;
		//dbg << F("u: ") << menu_item << '\n';
	}

	if (menu_select == 1) {
		if (menu_item == 1) {
			settings.mode += 1;
			if (settings.mode > 3) settings.mode = 0;
		}
		if (menu_item == 2) {
			settings.fire_speed += 5;
			if (settings.fire_speed  > 100) settings.fire_speed = 100;
		}
	}

	display_status();
	encoder_timeout.set(5000);
}

void encoder_down(int8_t x) {
	if (menu_select == 0) {
		if (menu_item == 0) menu_item = 2;
		else menu_item--;
		encoder_timeout.set(10000);
		//dbg << F("d: ") << menu_item << '\n';
	}

	if (menu_select == 1) {
		if (menu_item == 1) {
			if (settings.mode == 0) settings.mode = 3;
			else settings.mode -= 1;
		}
		if (menu_item == 2) {
			settings.fire_speed -= 5;
			if (settings.fire_speed  < 50) settings.fire_speed = 50;
		}
	}

	display_status();
	encoder_timeout.set(10000);
}

void encoder_button(int8_t x) {
	if (x != 2) return;														// we use only a button press event
	if (!menu_item) return;													// no item is selected
	menu_select++;															// increase the select

	if (menu_select >= 2) {													// menu select 2 means, we are in edit mode
		set_eeprom(2, 7, &settings);										// write the settings
		menu_select = 0;													// back for a new select
	}
	//dbg << F("p: ") << x << F(", ") << menu_select << '\n';

	display_status();
	encoder_timeout.set(5000);
}


void display_status() {

	u8g2.firstPage();														// reset the buffer page counter													

	do {																	// step through the different pages
		draw_battery(battery_level);										// write the battery level

		u8g2.setFont(u8g2_font_7x14B_tr);									// we use a different font for the menu

		u8g2.setCursor(0, 35);												// set the curser to line 35 of 64

		u8g2.print(status_line_item(1));									// get the status of the line item
		u8g2.print(F("Mode: "));
		if (settings.mode == 0) u8g2.print(F("unlimited"));
		if (settings.mode == 1) u8g2.print(F("single"));
		if (settings.mode == 2) u8g2.print(F("double"));
		if (settings.mode == 3) u8g2.print(F("tripple"));

		u8g2.setCursor(0, 55);

		u8g2.print(status_line_item(2));									// get the status of the line item
		u8g2.print(F("Speed: "));											
		u8g2.print(settings.fire_speed);
		u8g2.print("%");

	} while (u8g2.nextPage());												// step throug the buffer pages till the end

	//dbg << F("status display update ") << _TIME << '\n';
}

char status_line_item(uint8_t item_nr) {
	if (item_nr == menu_item) {												// seems the item is selected
		if (menu_select) {													// seems it is commited to change the value
			return '#';
		} else {															// not commited for a change
			return '>';
		}

	} else {																// other item, return a blank
		return ' ';
	}

}

/* draws the welcome screen on the display, battery level follows later */
void display_welcome() {
	u8g2.clear();
	//u8g2.clearDisplay();													// clear the display
	u8g2.firstPage();														// start with the first buffer

	do {																	// loop throug all buffers
		u8g2.drawXBMP(0, 0, 128, 64, testbmp_bits);							// and draw the bitmap
	} while (u8g2.nextPage());												// till the last page
	dbg << F("show welcome screen ") << _TIME << '\n';						// some debug
}


/* display battery update writes only into the first 8 lines into the display, evverything
** else in the display is untouched. function is called by the battery measurement function in the main loop */
void display_battery_update(uint8_t level) {
	static uint8_t battery_level_display;
	if (battery_level_display == level) return;								// nothing to do

	u8g2.firstPage();														// start with the first page
	draw_battery(level);													// draw the status in the buffer
	u8g2.nextPage();														// write it to the display

	battery_level_display = level;											// remember on the new value
	dbg << F("update battery level to ") << level << F("% ") << _TIME << '\n';// some debug
}


void draw_battery(uint8_t battery) {
	// displaying the battery level uses the first 8 lines of the display 
	u8g2.setDrawColor(1);													// sets the color white for the frame and the box
	u8g2.drawFrame(0, 0, 100, 6);											// draws the frame
	u8g2.drawBox(0, 0, battery, 6);											// fills the frame

	u8g2.setFont(u8g2_font_6x12_tr);										// get the font into the memory
	uint8_t xpos = 117;														// battery info should be always at the same position, independend of the levvel
	if (battery >= 10) xpos = 111;											// differnet pos while 2 digits
	if (battery >= 100) xpos = 105;											// again with 3 digits
	u8g2.setCursor(xpos, 8);	
	u8g2.print(battery, DEC);												// and writes the battery level
	u8g2.print("%");														// 
}



ISR(TIMER0_COMPA_vect) {
	++milliseconds;
	encoder.service();
}


