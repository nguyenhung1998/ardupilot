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

TinyGPSPlus tinygps;

//GPVTG
TinyGPSCustom GPVTG_magnetic_course(tinygps, "GPVTG", 3);  // deg
TinyGPSCustom GPVTG_speed(tinygps, "GPVTG", 7);  // km/h

//PHROT
TinyGPSCustom PHROT_roll(tinygps, "PHROT", 1);  // deg/s
TinyGPSCustom PHROT_pitch(tinygps, "PHROT", 2);  // deg/s
TinyGPSCustom PHROT_heading(tinygps, "PHROT", 3);  // deg/s

//PHSPD
TinyGPSCustom PHSPD_surge(tinygps, "PHSPD", 1);  // m/s
TinyGPSCustom PHSPD_sway(tinygps, "PHSPD", 2);  // m/s
TinyGPSCustom PHSPD_heave(tinygps, "PHSPD", 3);  // m/s

//PHTRH
TinyGPSCustom PHTRH_pitch(tinygps, "PHTRH", 1);  // deg
TinyGPSCustom PHTRH_roll(tinygps, "PHTRH", 3);  // deg

//PHTRO
TinyGPSCustom PHTRO_pitch(tinygps, "PHTRO", 1);  // deg
TinyGPSCustom PHTRO_roll(tinygps, "PHTRO", 3);  // deg

//HEHDT
TinyGPSCustom HEHDT_heading(tinygps, "HEHDT", 1);  // deg

//HEROT
TinyGPSCustom HEROT_heading(tinygps, "HEROT", 1);  // deg/mn

//HETHS
TinyGPSCustom HETHS_heading(tinygps, "HETHS", 1);  // deg

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
		if (tinygps.encode((char)hal.uartB->read()))
		//char inChar = (char)hal.uartB->read();
		if (tinygps.location.isUpdated())
		{
			hal.uartA->printf("Location: %4.6f, %4.6f \n" , tinygps.location.lat(), tinygps.location.lng());
		}
		if (PHROT_roll.isUpdated())
		{
			hal.uartA->printf("PHROT: %s \n", PHROT_roll.value());
		}
		if (PHSPD_surge.isUpdated())
		{
			hal.uartA->printf("PHSPD: %s \n", PHSPD_surge.value());
		}
		if (PHTRH_pitch.isUpdated())
		{
			hal.uartA->printf("PHTRH: %s \n", PHTRH_pitch.value());
		}
		if (PHTRH_pitch.isUpdated())
		{
			hal.uartA->printf("PHTRH: %s \n", PHTRH_pitch.value());
		}
		if (PHTRO_pitch.isUpdated())
		{
			hal.uartA->printf("PHTRO_pitch: %s \n", PHTRO_pitch.value());
		}
		if (HEHDT_heading.isUpdated())
		{
			hal.uartA->printf("HEHDT: %s \n", HEHDT_heading.value());
		}
	}
}

AP_HAL_MAIN();
