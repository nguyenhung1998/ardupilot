#ifndef _AP_NEW_SENSOR_H_
#define _AP_NEW_SENSOR_H_


#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

class New_Sensor{
	public:
		New_Sensor();

//		void Info();

		void Get_DataFrame(char *str);
		/*
			Example frame
				{data1,data2,data3,data4,.....,data28}
		*/

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
		//void Get_GPS_location(int32_t &lat, int32_t &lng,int32_t &alt);
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
		void Read_Data();
		void Read_GPS_Data(int32_t lat, int32_t lng, int32_t alt);

		/*
			Example Get
				float euler_x, euler_y, euler_z;
				Get_Euler(euler_x, euler_y, euler_z);
		*/

	private:
		/*
		float accelNED_x, accelNED_y, accelNED_z; 	-> sensor[0:2]
		float euler_x, euler_y, euler_z;			-> sensor[3:5]
		float gyroBias_x, gyroBias_y, gyroBias_z;	-> sensor[6:8]
		float magNED_x, magNED_y, magNED_z;			-> sensor[9:11]
		float magXYZ_x, magXYZ_y, magXYZ_z;			-> sensor[12:14]
		float posNE_x, posNE_y, posD;				-> sensor[15:17]
		float ret_q1, ret_q2, ret_q3, ret_q4;		-> sensor[18:21]
		float vel_x, vel_y, vel_z;					-> sensor[22:24]
		float win_x, win_y;							-> sensor[25:26]
		float zbias;								-> sensor[27]
		*/
		double sensor[32];
		int32_t lat = 0;
		int32_t lng = 0;
		int32_t alt = 0;
		Vector3f gyro = Vector3f(0,0,0);
		Vector3f accel = Vector3f(0,0,0);
		float climb_rate = 0;
		float baro_altitude = 0;

};

void String_nCopy(char *src, char *dest, uint16_t start, uint16_t end);
void Vir_Data(char *str);


extern New_Sensor new_sensor;
#endif
