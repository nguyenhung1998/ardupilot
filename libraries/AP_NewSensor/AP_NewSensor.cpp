#include "AP_NewSensor.h"
extern const AP_HAL::HAL& hal;
New_Sensor new_sensor;

// Constructor New_Sensor
New_Sensor::New_Sensor()
{
	uint8_t i;
	for(i=0; i<32; i++)
		this->sensor[i] = 0;
}

// New_Sensor::Info
// View all sensor value
//void New_Sensor::Info()
//{
//	uint8_t i;
//	cout<<"New_Sensor Info"<<endl;
//	for(i=0; i<28; i++)
//	{
//		cout<<"sensor["<<int(i)<<"]: "<<this->sensor[i]<<"  ";
//		if((i+1)%5==0)
//			cout<<endl;
//	}
//	cout<<endl<<"-----------------"<<endl<<endl;
//}

// New_Sensor::Get_DataFrame
// Split and handling data from string dataFrame
void New_Sensor::Get_DataFrame(char *str)
{
	int16_t i = 0;
	uint16_t start = 0;
	uint16_t len = strlen(str);
	int8_t j = 0;

	char tmp_str[10];
	float tmp_sensor[30];

	for(i=0; i<len; i++)
	{
		if(str[i]==',')
		{
			String_nCopy(str, tmp_str, start, i-1);
			tmp_sensor[j++] = atof(tmp_str);
			start = i+1;
		}
	}
	if(start<len-1)
	{
		String_nCopy(str, tmp_str, start, len);
		tmp_sensor[j++] = atof(tmp_str);
	}
	if(j<30)
		return;
	else
	{
		while(j>=0)
		{
			j--;
			this->sensor[j] = tmp_sensor[j];
		}
	}
}

/*
	Get Data
*/

// Get_AccelNED
void New_Sensor::Get_AccelNED(float &accelNED_x, float &accelNED_y, float &accelNED_z)
{
	accelNED_x = this->sensor[0];
	accelNED_y = this->sensor[1];
	accelNED_z = this->sensor[2];
}

// Get_Euler
void New_Sensor::Get_Euler(float &euler_x, float &euler_y, float &euler_z)
{
	euler_x = this->sensor[3];
	euler_y = this->sensor[4];
	euler_z = this->sensor[5];
}

// Get_GyroBias
void New_Sensor::Get_GyroBias(float &gyroBias_x, float &gyroBias_y, float &gyroBias_z)
{
	gyroBias_x = this->sensor[6];
	gyroBias_y = this->sensor[7];
	gyroBias_z = this->sensor[8];
}

// Get_MagNED
void New_Sensor::Get_MagNED(float &magNED_x, float &magNED_y, float &magNED_z)
{
	magNED_x = this->sensor[9];
	magNED_y = this->sensor[10];
	magNED_z = this->sensor[11];
}

// Get_MagXYZ
void New_Sensor::Get_MagXYZ(float &magXYZ_x, float &magXYZ_y, float &magXYZ_z)
{
	magXYZ_x = this->sensor[12];
	magXYZ_x = this->sensor[13];
	magXYZ_x = this->sensor[14];
}

// Get_PosNE
void New_Sensor::Get_PosNE(float &posNE_x, float &posNE_y)
{
	posNE_x = this->sensor[15];
	posNE_y = this->sensor[16];
}

//Get PosD
void New_Sensor::Get_PosD(float &posD)
{
	posD = this->sensor[17];
}

// Get_Ret
void New_Sensor::Get_Ret(float &ret_q1, float &ret_q2, float &ret_q3, float &ret_q4)
{
	ret_q1 = this->sensor[18];
	ret_q2 = this->sensor[19];
	ret_q3 = this->sensor[20];
	ret_q4 = this->sensor[21];
}

// Get_Vel
void New_Sensor::Get_Vel(float &vel_x, float &vel_y, float &vel_z)
{
	vel_x = this->sensor[22];
	vel_y = this->sensor[23];
	vel_z = this->sensor[24];
}

// Get_Win
void New_Sensor::Get_Win(float &win_x, float &win_y)
{
	win_x = this->sensor[25];
	win_y = this->sensor[26];
}

// Get_Zbias
void New_Sensor::Get_Zbias(float &zbias)
{
	zbias = this->sensor[27];
}

void New_Sensor::Get_GPS_stuff(uint16_t &time_week, uint32_t &time_week_ms, uint8_t &num_sats, uint32_t &last_gps_time_ms)
{
	time_week = 1721;
	time_week_ms =AP_HAL::millis() + 3*60*60*1000 + 37000;
	num_sats = 3;
	last_gps_time_ms = AP_HAL::millis();
}

//void New_Sensor::Get_GPS_location(uint32_t &lat, uint32_t &lng, uint32_t &alt)
//{
//	lat = 209727828L;
//	lng = 1057774111L;
//	alt = 58400;
//}

int32_t New_Sensor::Get_GPS_lat()
{
	//int32_t lat = 209727828L;
//	return this->sensor[28];
	return this->lat;
}

int32_t New_Sensor::Get_GPS_lng()
{
	//int32_t lng = 1057774111L;
//	return this->sensor[29];
	return this->lng;
}

int32_t New_Sensor::Get_GPS_alt()
{
	//int32_t alt = 58400;
//	return this->sensor[30];
	return this->alt;
}

void New_Sensor::Read_GPS_Data(int32_t new_lat, int32_t new_lng, int32_t new_alt)
{
//	this->sensor[28] = lat;
//	this->sensor[29] = lng;
//	this->sensor[30] = alt;
	this->lat = new_lat;
	this->lng = new_lng;
	this->alt = new_alt;
}

void New_Sensor::Get_GPS_velocity(float &x, float &y, float &z)
{
	x = 100;
	y = 100;
	z = 100;
}

void New_Sensor::Get_GPS_ground(float &ground_speed, float &ground_course)
{
	ground_course = 18000000;
	ground_speed = 111;
}

void New_Sensor::Get_GPS_accuracy(float &speed_accuracy, float &horizontal_accuracy, float &vertical_accuracy)
{
	speed_accuracy = 0;
	horizontal_accuracy = 0;
	vertical_accuracy = 0;
}

void New_Sensor::Get_GPS_dilution(uint16_t &hdop, uint16_t &vdop)
{
	hdop = 0;
	vdop = 0;
}

void New_Sensor::Read_Data()
{
	char message[200];
	int len_message = 0;
	bool recieved = true;
	while(recieved)
	{

		while(hal.uartB->available())
		{
			char inChar = (char)hal.uartB->read();
			if (inChar == '\n')
			{
				break;
			}
			message[len_message] = inChar;
			len_message++;
			if (len_message > 200)
			{
				break;
			}

		}

		if (len_message > 200)
		{
			break;
		}

		if(len_message > 0)
		{
			new_sensor.Get_DataFrame(message);
			recieved = false;
		}
	}
}

void String_nCopy(char *src, char *dest, uint16_t start, uint16_t end)
{
	uint16_t i, j;
	dest[0] = '\0';
	j = 0;
	for(i=start; i<=end; i++)
	{
		dest[j++] = src[i];
	}
	dest[j] = '\0';
}

void Vir_Data(char *str)
{
	uint8_t i = 0;
	uint8_t j = 0;
	uint8_t sign = 0;
	str[0] = '\0';
	for(j=0; j<28; j++)
	{
		sign = rand()%2;
		if(sign==1)
		{
			str[i++] ='-';
		}
		str[i++] = '0';
		str[i++] = '.';
		str[i++] = rand()%10 + 48;
		if(j<27)
			str[i++] = ',';
	}
	str[i] = '\0';
}


