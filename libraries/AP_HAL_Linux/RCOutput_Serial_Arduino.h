#pragma once

#include "AP_HAL_Linux.h"

class RCOutput_Serial_Arduino : public AP_HAL::RCOutput {
    void     init();
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);
    void     cork(void) override {}
    void     push(void) override {}

	virtual bool force_safety_on(void);


     virtual void     force_safety_off(void);

private:
    static const uint8_t channel_count = 12;
    uint16_t period[channel_count];

    void update();
};

struct Odrive_Axis_State {
	float output_normalized;
	float position;
	float speed;
	float current;
	uint32_t last_update;
};



class Odrive {
public:
	inline void get_encoder(int axis, float* position, float* speed) {
		*position = _motor_states[axis].position;
		*speed = _motor_states[axis].speed;
	}

	inline uint32_t get_last_update_time(int axis) {return _motor_states[axis].last_update;}
	inline void set_output(int axis, float value) {
		_motor_states[axis].output_normalized = value;
	}
	inline float get_voltage() {
		return bus_voltage;
	}
	inline float get_current() {
		return _motor_states[0].current + _motor_states[1].current;
	}



	void update();

	void init();

	void run();


	//void init();

private:
	struct Odrive_Axis_State _motor_states[2];

	float bus_voltage;

	void enable();
	void disable();

	pthread_t _thread;
	static void *_control_thread(void *arg);
};

Odrive &get_odrive();
