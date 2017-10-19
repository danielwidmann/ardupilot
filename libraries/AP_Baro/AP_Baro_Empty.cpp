#include "AP_Baro_Empty.h"



AP_Baro_Empty::AP_Baro_Empty(AP_Baro &baro): AP_Baro_Backend(baro) {
	_instance = _frontend.register_sensor();
}


void AP_Baro_Empty::update() {
	_copy_to_frontend(0, 1015, 30);
}
