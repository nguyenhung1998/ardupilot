#ifndef _AP_NEW_SENSOR_H_
#define _AP_NEW_SENSOR_H_


#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

class New_Sensor{
	public:
		void Get_AccelNED(float &accelNED_x, float &accelNED_y, float &accelNED_z);
		void Get_Euler(float &euler_x, float &euler_y, float &euler_z);
		void Get_GyroBias(float &gyroBias_x, float &gyroBias_y, float &gyroBias_z);
		void Get_MagNED(float &magNED_x, float &magNED_y, float &magNED_z);
		void Get_MagXYZ(float &magXYZ_x, float &magXYZ_y, float &magXYZ_z);
		void Get_PosNE(float &posNE_x, float &posNE_y);
		void Get_PosD(float &posD);
		void Get_Ret(float &ret_q1, float &ret_q2, float &ret_q3, float &ret_q4);
		void Get_Vel(float &vel_x, float &vel_y, float &vel_z);
		void Get_Win(float &win_x, float &win_y);
		void Get_Zbias(float &zbias);
		void Get_GPS_stuff(uint16_t &time_week, uint32_t &time_week_ms, uint8_t &num_sats, uint32_t &last_gps_time_ms);
		int32_t Get_GPS_lat();
		int32_t Get_GPS_lng();
		int32_t Get_GPS_alt();
		void Get_GPS_velocity(float &x, float &y, float &z);
		void Get_GPS_ground(float &ground_speed, float &ground_course);
		void Get_GPS_accuracy(float &speed_accuracy, float &horizontal_accuracy, float &vertical_accuracy);
		void Get_GPS_dilution(uint16_t &hdop, uint16_t &vdop);
		Vector3f Get_accel();
		Vector3f Get_gyro();
		float Get_climb_rate();
		float Get_baro_altitude();
		void Read_GPS_Data(int32_t lat, int32_t lng, int32_t alt);

		void Read_AccelNED(float accelNED_x, float accelNED_y, float accelNED_z);
		void Read_Euler(float euler_x, float euler_y, float euler_z);
		void Read_GyroBias(float gyroBias_x, float gyroBias_y, float gyroBias_z);
		void Read_MagNED(float magNED_x, float magNED_y, float magNED_z);
		void Read_MagXYZ(float magXYZ_x, float magXYZ_y, float magXYZ_z);
		void Read_PosNED(float posNE_x, float posNE_y, float posD);
		void Read_Ret(float ret_q1, float ret_q2, float ret_q3, float ret_q4);
		void Read_Vel(float vel_x, float vel_y, float vel_z);
		void Read_Win(float win_x, float win_y);
		void Read_Zbias(float zbias);
		void Read_GPS_stuff(uint16_t time_week, uint32_t time_week_ms, uint32_t last_gps_time_ms);
		//void Get_GPS_location(int32_t &lat, int32_t &lng,int32_t &alt);

		void Read_GPS_velocity(float x, float y, float z);
		void Read_GPS_ground(float ground_speed, float ground_course);
		void Read_GPS_accuracy(float speed_accuracy, float horizontal_accuracy, float vertical_accuracy);
		void Read_GPS_dilution(uint16_t hdop, uint16_t vdop);
		void Read_accel(float accel_x, float accel_y, float accel_z);
		void Read_gyro(float gyro_x, float gyro_y, float gyro_z);
		void Read_climb_rate_altitude(float climb_rate, float baro_altitude);
		/*
			Example Get
				float euler_x, euler_y, euler_z;
				Get_Euler(euler_x, euler_y, euler_z);
		*/

	private:
		float accelNED_x, accelNED_y, accelNED_z = 0;
		float euler_x, euler_y, euler_z = 0;
		float gyroBias_x, gyroBias_y, gyroBias_z = 0;
		float magNED_x, magNED_y, magNED_z = 0;
		float magXYZ_x, magXYZ_y, magXYZ_z = 0;
		float posNE_x, posNE_y, posD = 0;
		float ret_q1, ret_q2, ret_q3, ret_q4 = 0;
		float vel_x, vel_y, vel_z = 0;
		float win_x, win_y = 0;
		float zbias = 0;

		int32_t lat = 0;
		int32_t lng = 0;
		int32_t alt = 0;
		float gps_vel_x = 0;
		float gps_vel_y = 0;
		float gps_vel_z = 0;
		float ground_course = 0;
		float ground_speed = 0;
		float speed_accuracy = 0;
		float horizontal_accuracy = 0;
		float vertical_accuracy = 0;
		uint16_t hdop = 0;
		uint16_t vdop = 0;
		uint16_t time_week = 1721;
		uint32_t time_week_ms = AP_HAL::millis() + 3*60*60*1000 + 37000;
		uint8_t num_sats = 3;
		uint32_t last_gps_time_ms = AP_HAL::millis();

		Vector3f gyro = Vector3f(0,0,0);
		Vector3f accel = Vector3f(0,0,0);
		float climb_rate = 0;
		float baro_altitude = 0;



};


extern New_Sensor new_sensor;
#endif
