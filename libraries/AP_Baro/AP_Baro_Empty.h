#pragma once

#include "AP_Baro_Backend.h"

class AP_Baro_Empty: public AP_Baro_Backend
{
public:
    AP_Baro_Empty(AP_Baro &baro);

    virtual void update();
private:
    uint8_t _instance;
};
