#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#include "AP_Baro_NewSensor.h"
extern const AP_HAL::HAL& hal;

/*
  constructor - registers instance at top Baro driver
 */
AP_Baro_NewSensor::AP_Baro_NewSensor(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
	_instance = _frontend.register_sensor();
#if APM_BUILD_TYPE(APM_BUILD_ArduSub)
        _frontend.set_type(_instance, AP_Baro::BARO_TYPE_WATER);
#endif
}



// Read the sensor
void AP_Baro_NewSensor::update(void)
{
    _recent_press = 1;
    _recent_temp = 1;

    WITH_SEMAPHORE(_sem);
    _copy_to_frontend(_instance, _recent_press, _recent_temp);
}

