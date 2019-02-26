#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_NewSensor.h"
#include <AP_NewSensor/AP_NewSensor.h>
#include <stdio.h>

const extern AP_HAL::HAL& hal;

AP_InertialSensor_NewSensor::AP_InertialSensor_NewSensor(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu)
{
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_NewSensor::detect(AP_InertialSensor &_imu)
{
    AP_InertialSensor_NewSensor *sensor = new AP_InertialSensor_NewSensor(_imu);
    if (sensor == nullptr) {
        return nullptr;
    }
    if (!sensor->init_sensor()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_InertialSensor_NewSensor::init_sensor(void)
{
    // grab the used instances
	gyro_instance[0] = _imu.register_gyro(1000, AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SITL, 0, 1, DEVTYPE_SITL));
	accel_instance[0] = _imu.register_accel(1000, AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SITL, 0, 2, DEVTYPE_SITL));

	hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_NewSensor::timer_update, void));
	return true;
}

/*
  generate an accelerometer sample
 */
void AP_InertialSensor_NewSensor::generate_accel(uint8_t instance)
{

    Vector3f accel = new_sensor.Get_accel();

    _rotate_and_correct_accel(accel_instance[instance], accel);
    
    _notify_new_accel_raw_sample(accel_instance[instance], accel, AP_HAL::micros64());
}

/*
  generate a gyro sample
 */
void AP_InertialSensor_NewSensor::generate_gyro(uint8_t instance)
{
	Vector3f gyro = new_sensor.Get_gyro();

    _rotate_and_correct_gyro(gyro_instance[instance], gyro);
    
    _notify_new_gyro_raw_sample(gyro_instance[instance], gyro, AP_HAL::micros64());
}

void AP_InertialSensor_NewSensor::timer_update(void)
{
    uint64_t now = AP_HAL::micros64();
#if 0
    // insert a 1s pause in IMU data. This triggers a pause in EK2
    // processing that leads to some interesting issues
    if (now > 5e6 && now < 6e6) {
        return;
    }
#endif

	if (now >= next_accel_sample[0]) {
		generate_accel(0);
		while (now >= next_accel_sample[0]) {
			next_accel_sample[0] += 1000000UL / 1000;
		}
	}
	if (now >= next_gyro_sample[0]) {
		generate_gyro(0);
		while (now >= next_gyro_sample[0]) {
			next_gyro_sample[0] += 1000000UL / 1000;
		}
	}

}

bool AP_InertialSensor_NewSensor::update(void)
{
    update_accel(accel_instance[0]);
    update_gyro(gyro_instance[0]);
    return true;
}
