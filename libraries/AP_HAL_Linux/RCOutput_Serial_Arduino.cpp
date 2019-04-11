#include "RCOutput_Serial_Arduino.h"
#include "stdio.h"
#include <unistd.h>

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

uint16_t state[16];

void RCOutput_Serial_Arduino::init() {
	hal.uartF->begin(9600);

//	usleep(1000 * 1000);
//	// reset odrive
//	hal.uartF->printf("\n");
//	hal.uartF->printf("sb\n");
//	usleep(1000 * 1000);
}

void RCOutput_Serial_Arduino::set_freq(uint32_t chmask, uint16_t freq_hz) {
}

uint16_t RCOutput_Serial_Arduino::get_freq(uint8_t ch) {
	return 50;
}

void RCOutput_Serial_Arduino::enable_ch(uint8_t ch) {
}

void RCOutput_Serial_Arduino::disable_ch(uint8_t ch) {
}

 bool RCOutput_Serial_Arduino::force_safety_on(void) {
	printf("safty on\n");
	hal.uartF->printf("\n");

	// go to idle mode
	hal.uartF->printf("w axis0.requested_state 1\n");

	return true;
}

 void RCOutput_Serial_Arduino::force_safety_off(void) {
	printf("safty off\n");
	//		usleep(1000 * 1000);

	// clear errors
			hal.uartF->printf("\n");
			hal.uartF->printf("w axis0.motor.error 0\n");
			hal.uartF->printf("w axis0.error 0\n");

			// start closed loop controlodrv0.axis0.error
			hal.uartF->printf("w axis0.requested_state 8\n");
}

void RCOutput_Serial_Arduino::update() {
	//hal.uartF->begin(9600);

	float normalized = (period[0] - 1500) / 400.0;
	float maxSpeed = 30;

	int armed = hal.util->get_soft_armed()? 1: 0;
	static	int lastArmed = 2;

	if(armed != lastArmed) {
		if(armed == 1)
			force_safety_off();
		else
			force_safety_on();
	}

	lastArmed = armed;



//	static bool reset = false;
//	if (!reset) {
//
//		reset = true;
////		hal.uartF->printf("\n");
////		hal.uartF->printf("sb\n");
//		printf("reset motor 4\n");
////		usleep(1000 * 1000);
//
//// clear errors
//		hal.uartF->printf("\n");
//		hal.uartF->printf("w axis0.motor.error 0\n");
//		hal.uartF->printf("w axis0.error 0\n");
//
//		// start closed loop controlodrv0.axis0.error
//		hal.uartF->printf("w axis0.requested_state 8\n");
//
//		return;
//	}

	// update motor speeds
	hal.uartF->printf("v 0 %f\n", normalized * maxSpeed);

	//period[2]

	// read encoder feedback
	hal.uartF->printf("f 0\n");
	char buffer[40];
	memset(buffer, 0, sizeof(buffer));
	char* current = buffer;
	while (true) {
		int value = hal.uartF->read();
		*(current++) = value;
		if (value == '\n' || current >= &buffer[sizeof(buffer) - 2])
			break;
	}

	char * s1 = strtok(buffer, " ");
	char * s2 = strtok(NULL, " ");

	if (s1 == NULL || s2 == NULL)
		return;
	//*s1 = '\0';
	float pos = atof(s1);
	float speed = atof(s2);

	(void) pos;
	(void) speed;

	//printf("%s %f %f\n", buffer, pos, speed);

	static int counter = 0;

	if (counter++ > 100) {
		counter = 0;
		printf("v 0 %f\n", normalized * maxSpeed);
		printf("%s %f %f\n", buffer, pos, speed);
	}

}

void RCOutput_Serial_Arduino::write(uint8_t ch, uint16_t period_us) {
	if (ch >= channel_count) {
		return;
	}

	period[ch] = period_us;

	static int counter = 0;

	if (counter++ > 8) {
		counter = 0;
	} else {
		return;
	}

	update();
}

uint16_t RCOutput_Serial_Arduino::read(uint8_t ch) {
	if (ch >= channel_count) {
		return 0;
	}
	return period[ch];
}

void RCOutput_Serial_Arduino::read(uint16_t* period_us, uint8_t len) {
	for (int i = 0; i < len; i++) {
		period_us[i] = read(i);
	}
}

