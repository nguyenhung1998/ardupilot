#include "AP_NewSensor.h"
extern const AP_HAL::HAL& hal;
New_Sensor new_sensor;

// Get_AccelNED
void New_Sensor::Get_AccelNED(float &accelNED_x_new, float &accelNED_y_new, float &accelNED_z_new)
{
	accelNED_x_new = this->accelNED_x;
	accelNED_y_new = this->accelNED_y;
	accelNED_z_new = this->accelNED_z;
}

// Get_Euler
void New_Sensor::Get_Euler(float &euler_x_new, float &euler_y_new, float &euler_z_new)
{
	euler_x_new = this->euler_x;
	euler_y_new = this->euler_y;
	euler_z_new = this->euler_z;
}

// Get_GyroBias
void New_Sensor::Get_GyroBias(float &gyroBias_x_new, float &gyroBias_y_new, float &gyroBias_z_new)
{
	gyroBias_x_new = this->gyroBias_x;
	gyroBias_y_new = this->gyroBias_y;
	gyroBias_z_new = this->gyroBias_z;
}

// Get_MagNED
void New_Sensor::Get_MagNED(float &magNED_x_new, float &magNED_y_new, float &magNED_z_new)
{
	magNED_x_new = this->magNED_x;
	magNED_y_new = this->magNED_y;
	magNED_z_new = this->magNED_z;
}

// Get_MagXYZ
void New_Sensor::Get_MagXYZ(float &magXYZ_x_new, float &magXYZ_y_new, float &magXYZ_z_new)
{
	magXYZ_x_new = this->magXYZ_x;
	magXYZ_y_new = this->magXYZ_y;
	magXYZ_z_new = this->magXYZ_z;
}

// Get_PosNE
void New_Sensor::Get_PosNE(float &posNE_x_new, float &posNE_y_new)
{
	posNE_x_new = this->posNE_x;
	posNE_y_new = this->posNE_y;
}

//Get PosD
void New_Sensor::Get_PosD(float &posD_new)
{
	posD_new = this->posD;
}

// Get_Ret
void New_Sensor::Get_Ret(float &ret_q1_new, float &ret_q2_new, float &ret_q3_new, float &ret_q4_new)
{
	ret_q1_new = this->ret_q1;
	ret_q2_new = this->ret_q2;
	ret_q3_new = this->ret_q3;
	ret_q4_new = this->ret_q4;
}

// Get_Vel
void New_Sensor::Get_Vel(float &vel_x_new, float &vel_y_new, float &vel_z_new)
{
	vel_x_new = this->vel_x;
	vel_y_new = this->vel_y;
	vel_z_new = this->vel_z;
}

// Get_Wind
void New_Sensor::Get_Win(float &win_x_new, float &win_y_new)
{
	win_x_new = this->win_x;
	win_y_new = this->win_y;
}

// Get_Zbias
void New_Sensor::Get_Zbias(float &zbias_new)
{
	zbias_new = this->zbias;
}

void New_Sensor::Get_GPS_stuff(uint16_t &time_week_new, uint32_t &time_week_ms_new, uint8_t &num_sats_new, uint32_t &last_gps_time_ms_new)
{
	time_week_new = 1721;
	time_week_ms_new =AP_HAL::millis() + 3*60*60*1000 + 37000;
	num_sats_new = 3;
	last_gps_time_ms_new = AP_HAL::millis();
}

//void New_Sensor::Get_GPS_location(uint32_t &lat, uint32_t &lng, uint32_t &alt)
//{
//	lat = 209727828L;
//	lng = 1057774111L;
//	alt = 58400;
//}

int32_t New_Sensor::Get_GPS_lat()
{
	return this->lat;
}

int32_t New_Sensor::Get_GPS_lng()
{
	return this->lng;
}

int32_t New_Sensor::Get_GPS_alt()
{
	return this->alt;
}

void New_Sensor::Get_GPS_velocity(float &x, float &y, float &z)
{
	x = this->gps_vel_x;
	y = this->gps_vel_y;
	z = this->gps_vel_z;
}

void New_Sensor::Get_GPS_ground(float &ground_speed_new, float &ground_course_new)
{
	ground_course_new = this->ground_course;
	ground_speed_new = this->ground_speed;
}

void New_Sensor::Get_GPS_accuracy(float &speed_accuracy_new, float &horizontal_accuracy_new, float &vertical_accuracy_new)
{
	speed_accuracy_new = this->speed_accuracy;
	horizontal_accuracy_new = this->horizontal_accuracy;
	vertical_accuracy_new = this->vertical_accuracy;
}

void New_Sensor::Get_GPS_dilution(uint16_t &hdop_new, uint16_t &vdop_new)
{
	hdop_new = this->hdop;
	vdop = this->vdop;
}

Vector3f New_Sensor::Get_accel()
{
	return this->accel;
}

Vector3f New_Sensor::Get_gyro()
{
	return this->gyro;
}

float New_Sensor::Get_climb_rate()
{
	return this->climb_rate;
}

float New_Sensor::Get_baro_altitude()
{
	return this->baro_altitude;
}

//Read

void New_Sensor::Read_GPS_Data(int32_t lat_new, int32_t lng_new, int32_t alt_new)
{
	this->lat = lat_new;
	this->lng = lng_new;
	this->alt = alt_new;
}

void New_Sensor::Read_AccelNED(float accelNED_x_new, float accelNED_y_new, float accelNED_z_new)
{
	this->accelNED_x = accelNED_x_new;
	this->accelNED_y = accelNED_y_new;
	this->accelNED_z = accelNED_z_new;
}

void New_Sensor::Read_Euler(float euler_x_new, float euler_y_new, float euler_z_new)
{
	this->euler_x = euler_x_new;
	this->euler_y = euler_y_new;
	this->euler_z = euler_z_new;
}

void New_Sensor::Read_GyroBias(float gyroBias_x_new, float gyroBias_y_new, float gyroBias_z_new)
{
	this->gyroBias_x = gyroBias_x_new;
	this->gyroBias_y = gyroBias_y_new;
	this->gyroBias_z = gyroBias_z_new;
}

void New_Sensor::Read_MagNED(float magNED_x_new, float magNED_y_new, float magNED_z_new)
{
	this->magNED_x = magNED_x_new;
	this->magNED_y = magNED_y_new;
	this->magNED_z = magNED_z_new;
}

void New_Sensor::Read_MagXYZ(float magXYZ_x_new, float magXYZ_y_new, float magXYZ_z_new)
{
	this->magXYZ_x = magXYZ_x_new;
	this->magXYZ_y = magXYZ_y_new;
	this->magXYZ_z = magXYZ_z_new;
}

void New_Sensor::Read_PosNED(float posNE_x_new, float posNE_y_new, float posD_new)
{
	this->posNE_x = posNE_x_new;
	this->posNE_y = posNE_y_new;
	this->posD = posD_new;
}

void New_Sensor::Read_Ret(float ret_q1_new, float ret_q2_new, float ret_q3_new, float ret_q4_new)
{
	this->ret_q1 = ret_q1_new;
	this->ret_q2 = ret_q2_new;
	this->ret_q3 = ret_q3_new;
	this->ret_q4 = ret_q4_new;
}

void New_Sensor::Read_Vel(float vel_x_new, float vel_y_new, float vel_z_new)
{
	this->vel_x = vel_x_new;
	this->vel_y = vel_y_new;
	this->vel_z = vel_z_new;
}

void New_Sensor::Read_Win(float win_x_new, float win_y_new)
{
	this->win_x = win_x_new;
	this->win_y = win_y_new;
}

void New_Sensor::Read_Zbias(float zbias_new)
{
	this->zbias = zbias_new;
}

void New_Sensor::Read_GPS_stuff(uint16_t time_week_new, uint32_t time_week_ms_new, uint32_t last_gps_time_ms_new)
{
	this->time_week = time_week_new;
	this->time_week_ms = time_week_ms_new;
	this->last_gps_time_ms = last_gps_time_ms_new;
}

void New_Sensor::Read_GPS_velocity(float x, float y, float z)
{
	this->gps_vel_x = x;
	this->gps_vel_y = y;
	this->gps_vel_z = z;
}

void New_Sensor::Read_GPS_ground(float ground_speed_new, float ground_course_new)
{
	this->ground_course = ground_course_new;
	this->ground_speed = ground_speed_new;
}

void New_Sensor::Read_GPS_accuracy(float speed_accuracy_new, float horizontal_accuracy_new, float vertical_accuracy_new)
{
	this->speed_accuracy = speed_accuracy_new;
	this->horizontal_accuracy = horizontal_accuracy_new;
	this->vertical_accuracy = vertical_accuracy_new;
}

void New_Sensor::Read_GPS_dilution(uint16_t hdop_new, uint16_t vdop_new)
{
	this->hdop = hdop_new;
	this->vdop = vdop_new;
}

void New_Sensor::Read_accel(float accel_x_new, float accel_y_new, float accel_z_new)
{
	this->accel.x = accel_x_new;
	this->accel.y = accel_y_new;
	this->accel.z = accel_z_new;
}
void New_Sensor::Read_gyro(float gyro_x_new, float gyro_y_new, float gyro_z_new)
{
	this->gyro.x = gyro_x_new;
	this->gyro.y = gyro_y_new;
	this->gyro.z = gyro_z_new;
}

void New_Sensor::Read_climb_rate_altitude(float climb_rate_new, float baro_altitude_new)
{
	this->climb_rate = climb_rate_new;
	this->baro_altitude = baro_altitude_new;
}



