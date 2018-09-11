
#include "RCOutput_Serial_Arduino.h"
#include "stdio.h"


static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

uint16_t state[16];

void RCOutput_Serial_Arduino::init() {
	hal.uartF->begin(9600);
}

void RCOutput_Serial_Arduino::set_freq(uint32_t chmask, uint16_t freq_hz) {}

uint16_t RCOutput_Serial_Arduino::get_freq(uint8_t ch) {
    return 50;
}

void RCOutput_Serial_Arduino::enable_ch(uint8_t ch)
{}

void RCOutput_Serial_Arduino::disable_ch(uint8_t ch)
{}

void RCOutput_Serial_Arduino::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= channel_count) {
        return;
    }

    period[ch] = period_us;

    static int counter = 0;

    if(counter++ > 8) {
    	counter = 0;
    } else
    {
    	return;
    }

    hal.uartF->begin(9600);
    hal.uartF->printf("%d %d\n", period[0]/2 - 1500/2, period[2]/2 - 1500/2);
}

uint16_t RCOutput_Serial_Arduino::read(uint8_t ch) {
    if (ch >= channel_count) {
        return 0;
    }
    return period[ch];
}

void RCOutput_Serial_Arduino::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}

