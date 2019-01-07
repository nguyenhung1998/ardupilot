/*
  simple test of UART interfaces
 */
#include <AP_HAL/AP_HAL.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <TinyGPS/TinyGPS++.h>

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// The TinyGPS++ object
TinyGPSPlus gps;

//GPVTG
TinyGPSCustom GPVTG_magnetic_course(gps, "GPVTG", 3);  // deg
TinyGPSCustom GPVTG_speed(gps, "GPVTG", 7);  // km/h

//PHROT
TinyGPSCustom PHROT_roll(gps, "PHROT", 1);  // deg/s
TinyGPSCustom PHROT_pitch(gps, "PHROT", 2);  // deg/s
TinyGPSCustom PHROT_heading(gps, "PHROT", 3);  // deg/s

//PHSPD
TinyGPSCustom PHSPD_surge(gps, "PHSPD", 1);  // m/s
TinyGPSCustom PHSPD_sway(gps, "PHSPD", 2);  // m/s
TinyGPSCustom PHSPD_heave(gps, "PHSPD", 3);  // m/s

//PHTRH
TinyGPSCustom PHTRH_pitch(gps, "PHTRH", 1);  // deg
TinyGPSCustom PHTRH_roll(gps, "PHTRH", 3);  // deg

//PHTRO
TinyGPSCustom PHTRO_pitch(gps, "PHTRO", 1);  // deg
TinyGPSCustom PHTRO_roll(gps, "PHTRO", 3);  // deg

//HEHDT
TinyGPSCustom HEHDT_heading(gps, "HEHDT", 1);  // deg

//HEROT
TinyGPSCustom HEROT_heading(gps, "HEROT", 1);  // deg/mn

//HETHS
TinyGPSCustom HETHS_heading(gps, "HETHS", 1);  // deg

/*
  setup one UART at 57600
 */
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->begin(115200);
}


void setup(void)
{
    /*
      start all UARTs at 57600 with default buffer sizes
    */

    hal.scheduler->delay(1000); //Ensure that the uartA can be initialized

    setup_uart(hal.uartA, "uartA");  // console
    setup_uart(hal.uartB, "uartB");  // console
//    setup_uart(hal.uartB, "uartB");  // 1st GPS
//    setup_uart(hal.uartC, "uartC");  // telemetry 1
//    setup_uart(hal.uartD, "uartD");  // telemetry 2
//    setup_uart(hal.uartE, "uartE");  // 2nd GPS
}

void loop(void)
{
	while(hal.uartB->available())
	{
		if (gps.encode((char)hal.uartB->read()))
		//char inChar = (char)hal.uartB->read();
		if (gps.location.isValid())
		{
			hal.uartA->printf("%4.6f, %4.6f \n" , gps.location.lat(), gps.location.lng());
		}
	}
}

AP_HAL_MAIN();
