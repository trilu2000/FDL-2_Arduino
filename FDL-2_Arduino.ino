// https://github.com/lexus2k/ssd1306
#include <ssd1306.h>

// https://github.com/0xPIT/encoder/tree/arduino
#include <ClickEncoder.h>

#include <Servo.h>
#include "myfunc.h"
#include "motors.h"

#define fdl2_battery    pinC3

#define fdl2_fire       pinD5


/* everything encoder related */
ClickEncoder encoder(pinC1, pinC0, pinC2, 4);
waittimer encoder_timer;
int16_t encoder_last, encoder_value;

/* battery related */
/* 892 = 12.35 Volt */
waittimer battery_timer;
uint32_t bat_value;

/* launcher to speed up the darts */
LauncherClass launcher(pinB2, 1000, 2000);

/* pusher to shift the dart into the launcher */
PusherClass pusher(pinB5, pinB4, pinB3, pinB1, pinD7, pinD6, launcher.ready);



void setup() {
	Serial.begin(115200);
	dbg << F("FDL-2 Arduino v0.1\n");							// init serial interface and some debug

	ssd1306_128x64_i2c_init();									// init the oled display
	ssd1306_fillScreen(0x00);									// and write some welcome text
	ssd1306_charF12x16(0, 1, "Welcome to", STYLE_NORMAL);
	ssd1306_charF12x16(25, 4, "FDL-2X", STYLE_BOLD);
	ssd1306_charF6x8(0, 7, "electronic by trilu");
	
	register_PCINT(fdl2_fire);									// init and register the fire button as interrupt

	analogReference(INTERNAL);									// battery reference to 1.1 Volt
	analogRead(fdl2_battery);
	battery_timer.set(1000);

	launcher.init();											// init the launcher

	// default settings
	launcher.fire_speed = 60;									// fire speed in % of max_speed
	launcher.speedup_time = 300;								// holds the time the motor needs to speedup
	launcher.standby_speed = 57;								// standby speed in % of max_speed
	launcher.standby_time = 500;								// standby time in ms
	pusher.mode = 2;											// how many darts per fire push

	//pci_ptr = &test_int;

}


void loop() {
	if (battery_timer.done()) {
		bat_value = analogRead(fdl2_battery);
		bat_value *= 1385;
		bat_value /= 1000;
		char x[5];
		itoa(bat_value, x, 10);
		ssd1306_charF6x8(100, 0, x, STYLE_NORMAL);
		battery_timer.set(5000);
	}

	if (encoder_timer.done()) {									// poll the encoder service function every 1 ms
		encoder.service();
		encoder_timer.set(1);
	}

	encoder_value += encoder.getValue();						// check if the encoder value had changed
	if (encoder_value > 255) encoder_value = 255;				// and take care of the changes
	if (encoder_value < 0) encoder_value = 0;

	if (encoder_value != encoder_last) {						// display the new encoder value
		encoder_last = encoder_value;
		dbg << encoder_value << '\n';
	}


	ClickEncoder::Button b = encoder.getButton();				// check the encoder button
	if (b != ClickEncoder::Open) {
		switch (b) {
			case ClickEncoder::Pressed:        dbg << F("pressed"); break;
			case ClickEncoder::Held:           dbg << F("held"); break;
			case ClickEncoder::Released:       dbg << F("released"); break;
			case ClickEncoder::Clicked:        dbg << F("clicked"); break;
			case ClickEncoder::DoubleClicked:  dbg << F("double clicked"); break;
		}
		dbg << '\n';
	}

	/* check fire button continously and drive pusher */
	uint8_t x = check_PCINT(fdl2_fire, 1);
	if (x == 2) {

		dbg << F("M::Fire!!\n");
		pusher.start();
		launcher.start();

	} else if (x == 3) {
		pusher.stop();
		launcher.stop();
	}
	pusher.poll();
	launcher.poll();






}






//void test_int(uint8_t vec, uint8_t pin, uint8_t flag) {
//	dbg << vec << ',' << pin << ',' << flag << ',' << millis() << '\n';
//}

/*void serialEvent() {

	while (Serial.available()) {
		uint8_t inChar = (uint8_t)Serial.read();			// read a byte
		if (inChar == 0x0A) return;

		if (isDigit(inChar)) {
			inString += (char)inChar;
		}

		if (inChar == 0x0D) {
			servo_angle = inString.toInt();
			//if (servo_angle > 179) servo_angle = 179;
			Serial.print("Received a new value: ");
			Serial.println(servo_angle);
			inString = "";
		}

	}
}*/