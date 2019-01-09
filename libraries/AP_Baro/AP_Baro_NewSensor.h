#pragma once

#include "AP_Baro_Backend.h"

#include <AP_Math/vectorN.h>

class AP_Baro_NewSensor : public AP_Baro_Backend {
public:
	AP_Baro_NewSensor(AP_Baro &);

    void update() override;

protected:

    void update_healthy_flag(uint8_t instance) override { _frontend.sensors[instance].healthy = true; }

private:
    uint8_t _instance;

    float _recent_temp;
    float _recent_press;

};

