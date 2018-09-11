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

private:
    static const uint8_t channel_count = 12;
    uint16_t period[channel_count];
};
