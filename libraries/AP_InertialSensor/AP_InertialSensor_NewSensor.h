#pragma once

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_NewSensor : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_NewSensor(AP_InertialSensor &imu);

    /* update accel and gyro state */
    bool update();

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

private:
    bool init_sensor(void);
    void timer_update();
    float gyro_drift(void);
    void generate_accel(uint8_t instance);
    void generate_gyro(uint8_t instance);

    // simulated sensor rates in Hz. This matches a pixhawk1
//    const uint16_t gyro_sample_hz[SENSOR_INSTANCES]  { 1000, 760 };
//    const uint16_t accel_sample_hz[SENSOR_INSTANCES] { 1000, 800 };
//
    uint8_t gyro_instance[1];
    uint8_t accel_instance[1];
    uint64_t next_gyro_sample[1];
    uint64_t next_accel_sample[1];
};
