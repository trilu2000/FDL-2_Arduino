// https://github.com/lexus2k/ssd1306
#include <ssd1306.h>

// https://github.com/0xPIT/encoder/tree/arduino
#include <ClickEncoder.h>

#include <Servo.h>
#include "myfunc.h"

#define fdl2_battery    pinC3

#define fdl2_enc_a      pinC1
#define fdl2_enc_b      pinC0
#define fdl2_enc_button pinC2

#define fdl2_fire       pinD5


/* everything encoder related */
ClickEncoder *encoder;
waittimer encoder_timer;
int16_t encoder_last, encoder_value;

/* battery related */
/* 892 = 12.35 Volt */
waittimer battery_timer;
uint32_t bat_value;

pusherclass pusher;

Servo myServo;													// create a servo object


void setup() {
	Serial.begin(115200);
	dbg << F("Servo tester\n");									// init serial interface and some debug

	ssd1306_128x64_i2c_init();									// init the oled display
	ssd1306_fillScreen(0x00);									// and write some welcome text
	ssd1306_charF12x16(0, 1, "Welcome to", STYLE_NORMAL);
	ssd1306_charF12x16(25, 4, "FDL-2X", STYLE_BOLD);
	ssd1306_charF6x8(0, 7, "electronic by trilu");
	
	register_PCINT(fdl2_fire);									// init and register the fire button as interrupt

	encoder = new ClickEncoder(fdl2_enc_a, fdl2_enc_b, fdl2_enc_button, 4); // init the encoder

	analogReference(INTERNAL);									// battery reference to 1.1 Volt
	analogRead(fdl2_battery);
	battery_timer.set(1000);

	pusher.ain2_pin = pinB4;									// introduce hardware pins to pusher class
	pusher.ain1_pin = pinB5;
	pusher.stby_pin = pinB1;
	pusher.pwma_pin = pinB3;
	pusher.back_sens = pinD6;
	pusher.frnt_sens = pinD7;
	pusher.init();												// init the pusher hardware, motor and switches
																
	myServo.attach(pinB2,1000,2000);							// attaches the servo on pin 10 to the servo object
	myServo.write(0);											// and set it off
	
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
		encoder->service();
		encoder_timer.set(1);
	}

	encoder_value += encoder->getValue();						// check if the encoder value had changed
	if (encoder_value > 255) encoder_value = 255;				// and take care of the changes
	if (encoder_value < 0) encoder_value = 0;

	if (encoder_value != encoder_last) {						// display the new encoder value
		encoder_last = encoder_value;
		dbg << encoder_value << '\n';
	}


	ClickEncoder::Button b = encoder->getButton();				// check the encoder button
	if (b != ClickEncoder::Open) {
		switch (b) {
			case ClickEncoder::Pressed:        dbg << "pressed"; break;
			case ClickEncoder::Held:           dbg << "held"; break;
			case ClickEncoder::Released:       dbg << "released"; break;
			case ClickEncoder::Clicked:        dbg << "clicked"; break;
			case ClickEncoder::DoubleClicked:  dbg << "double clicked"; break;
		}
		dbg << '\n';
	}

	/* check fire button continously and drive pusher */
	uint8_t x = check_PCINT(fdl2_fire, 1);
	if (x == 2) {
		//pusher.start();
		//pusher.count = 1;
		//pusher.round = 0;
		//if (myServo.read()) {
		//	myServo.write(0);
		//} else {
		dbg << "start\n";
		myServo.write(1100);
		delay(200);
		myServo.write(1500);
		delay(200);
		myServo.write(2000);
		//delay(200);
		pusher.start();
		pusher.count = 1;
		pusher.round = 0;
		//delay(200);
			//myServo.write(1832);
			//myServo.write(2150);
			//myServo.write(2150);

		//}

	} else if (x == 3) {
		pusher.stop();
		myServo.write(0);
	}
	pusher.poll();







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