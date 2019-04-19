#include "RCOutput_Serial_Arduino.h"
#include "stdio.h"
#include <unistd.h>

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

uint16_t state[16];

void RCOutput_Serial_Arduino::init() {
	hal.uartF->begin(9600);
	get_odrive().init();
	printf("init");

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

//	int armed = hal.util->get_soft_armed()? 1: 0;
//	static	int lastArmed = 2;
//
//	if(armed != lastArmed) {
//		if(armed == 1)
//			force_safety_off();
//		else
//			force_safety_on();
//	}

	//lastArmed = armed;



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

	if(ch == 0) {
		float normalized = (period_us - 1500) / 400.0;
		get_odrive().set_output(0, normalized);
	}

	if(ch == 2) {
		float normalized = (period_us - 1500) / 400.0;
		get_odrive().set_output(1, normalized);
	}

	static int counter = 0;

	if (counter++ > 8) {
		counter = 0;
	} else {
		return;
	}

	//update();
	//get_odrive().update();
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


// odrive


char* send_command(const char* command) {
	static char buffer[40];
	memset(buffer, 0, sizeof(buffer));

	// flush stale input
	while(hal.uartF->available() > 0) {
		hal.uartF->read();
	}

	hal.uartF->printf("%s\n", command);
	char* current = buffer;
	while (true) {
		int timeout = 0;
		while(hal.uartF->available() == 0) {
			usleep(1000);
			if(timeout++ > 10000) {
				return buffer;
			}
		}

		int value = hal.uartF->read();
		if(value == '\n' && current == buffer) {
			// ignore empty line
			continue;
		}

		// put character into output buffer
		*(current++) = value;

		if (value == '\n' || current >= &buffer[sizeof(buffer) - 2])
			break;
	}

	return buffer;

}


float read_param(const char* param_name) {
	char input[80];
	sprintf(input, "r %s\n", param_name);

	char* buffer = send_command(input);
	return atof(buffer);
}

void read_feedback(int motor, float* pos, float* speed) {
	char input[80];
	sprintf(input, "f %d\n", motor);

	char* buffer = send_command(input);

	char * s1 = strtok(buffer, " ");
	char * s2 = strtok(NULL, " ");

	if (s1 == NULL || s2 == NULL)
		return;

	*pos = atof(s1);
	*speed = atof(s2);
}

void Odrive::init() {
	struct sched_param param = { .sched_priority = 14 };
	pthread_attr_t attr;
	int ret;

	ret = pthread_attr_init(&attr);
	if (ret != 0) {
		perror("Odrive: failed to init attr\n");
		//goto exit;
	}
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
	pthread_attr_setschedparam(&attr, &param);

	ret = pthread_create(&_thread, &attr, _control_thread, this);
	if (ret != 0) {
		perror("ODrive: failed to create thread\n");
	}


}

void* Odrive::_control_thread(void *arg) {
	Odrive* cl = (Odrive *) arg;

	// make sure everything is started
	//sleep(2);

	cl->disable();

    while(true) {
    		cl->update();
    	}
}

void Odrive::run() {

}
void Odrive::enable() {
	printf("enable\n");

	// clear errors
	hal.uartF->printf("\n");
	hal.uartF->printf("w axis0.motor.error 0\n");
	hal.uartF->printf("w axis0.error 0\n");
	hal.uartF->printf("w axis1.motor.error 0\n");
	hal.uartF->printf("w axis1.error 0\n");

	// start closed loop controlodrv0.axis0.error
	hal.uartF->printf("w axis0.requested_state 8\n");
	hal.uartF->printf("w axis1.requested_state 8\n");

//	usleep(100000);

//	while (read_param("axis1.current_state") < 7.9)
//	{
//		usleep(1000);
//	}

}

void Odrive::disable() {
	printf("disable motors\n");
	hal.uartF->printf("\n");

	// go to idle mode
	hal.uartF->printf("w axis0.requested_state 1\n");
	hal.uartF->printf("w axis1.requested_state 1\n");
}



//float read_param(const char* param_name) {
//	char buffer[40];
//	memset(buffer, 0, sizeof(buffer));
//
//	//printf("r %s\n", param_name);
//	//hal.uartF->printf("r %s\n", param_name);
//
//	//hal.uartF->flush();
//	while(hal.uartF->available() > 0) {
//		hal.uartF->read();
//	}
//
//	hal.uartF->printf("r vbus_voltage\n");
//	char* current = buffer;
//	while (true) {
//		while(hal.uartF->available() == 0) {
//
//		}
//
//		int value = hal.uartF->read();
//		if(value == '\n' && current == buffer) {
//			// ignore empty line
//			continue;
//		}
//
//
//		*(current++) = value;
//
//		if (value == '\n' || current >= &buffer[sizeof(buffer) - 2])
//			break;
//	}
//
//	return atof(buffer);
//}

void Odrive::update() {
	float maxSpeed = 50;

	int armed = hal.util->get_soft_armed() ? 1 : 0;
	static int lastArmed = 2;

	if (armed != lastArmed) {
		if (armed == 1)
			enable();
		else
			disable();
	}

	lastArmed = armed;

	// update motor speeds
	hal.uartF->printf("v 0 %f\n", _motor_states[0].output_normalized * maxSpeed);
	hal.uartF->printf("v 1 %f\n", _motor_states[1].output_normalized * maxSpeed);

	//period[2]

	//hal.uartF->flush();
	// read encoder feedback
	//hal.uartF->printf("f 0\n");
//	char* buffer = send_command("f 0\n");
////	char buffer[40];
////	memset(buffer, 0, sizeof(buffer));
////	char* current = buffer;
////	while (true) {
////		int value = hal.uartF->read();
////		*(current++) = value;
////		if (value == '\n' || current >= &buffer[sizeof(buffer) - 2])
////			break;
////	}
//
//
//	char * s1 = strtok(buffer, " ");
//	char * s2 = strtok(NULL, " ");
//
//	if (s1 == NULL || s2 == NULL)
//		return;
//	//*s1 = '\0';
//	float pos = atof(s1);
//	float speed = atof(s2);
//
//	(void) pos;
//	(void) speed;
//
//	_motor_states[0].position = pos;
//	_motor_states[0].speed = speed;

	read_feedback(0, &_motor_states[0].position, &_motor_states[0].speed);
	read_feedback(1, &_motor_states[1].position, &_motor_states[1].speed);
	_motor_states[0].last_update = AP_HAL::millis();
	_motor_states[1].last_update = AP_HAL::millis();


	//bus_voltage = read_param("vbus_voltage");
	//_motor_states[0].current = read_param("axis0.motor.current_control.Iq_measured");
	//_motor_states[1].current = read_param("axis0.motor.current_control.Iq_measured");


	//printf("%s %f %f\n", buffer, pos, speed);

	// odrv0.axis0.motor.current_control.Iq_measured

	static int counter = 0;

	if (counter++ > 20) {
		bus_voltage = read_param("vbus_voltage");

		counter = 0;
		printf("v %f %f\n", _motor_states[0].output_normalized * maxSpeed, _motor_states[1].output_normalized * maxSpeed);
		printf("f %f %f %f %f\n", _motor_states[0].position, _motor_states[0].speed, _motor_states[1].position, _motor_states[1].speed);

		printf("Bus voltage:  %f %f\n", get_voltage(), get_current());
		//printf("err:  %f %f\n", read_param("axis0.error"), read_param("axis0.current_state"));
//		printf("Bus voltage:  %f\n", read_param("vbus_voltage"));
	}

}

//static bool initialized = false;
static Odrive odriveInstance;



Odrive &get_odrive()
{
//	if(!initialized){
//		initialized = true;
//		odriveInstance.init();
//	}
    return odriveInstance;
}


