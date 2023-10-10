#include "stdio.h"
#include "Globals.h"
#include "GPS/GPS.h"

extern UART_HandleTypeDef huart4;

uint8_t GPS_Init()
{
	HAL_UART_Receive_IT(&huart4, &Uart4Buffer, 1);

	return 0;
}

int GPS_validate(char *nmeastr)
{
	char check[3];
	char checkcalcstr[3];
	int i;
	int calculated_check;

	i = 0;
	calculated_check = 0;

	// check to ensure that the string starts with a $
	if (nmeastr[i] == '$')
		i++;
	else
		return 0;

	//No NULL reached, 75 char largest possible NMEA message, no '*' reached
	while ((nmeastr[i] != 0) && (nmeastr[i] != '*') && (i < 75))
	{
		calculated_check ^= nmeastr[i]; // calculate the checksum
		i++;
	}

	if (i >= 75)
	{
		return 0; // the string was too long so return an error
	}

	if (nmeastr[i] == '*')
	{
		check[0] = nmeastr[i + 1];    //put hex chars in check string
		check[1] = nmeastr[i + 2];
		check[2] = 0;
	}
	else
		return 0;    // no checksum separator found there for invalid

	sprintf(checkcalcstr, "%02X", calculated_check);
	return ((checkcalcstr[0] == check[0]) && (checkcalcstr[1] == check[1])) ?
			1 : 0;
}

void GPS_parse(char *GPSstrParse)
{
	if (!strncmp(GPSstrParse, "$GPGGA", 6))
	{
		if (sscanf(GPSstrParse, "$GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c",
				&GPS.utc_time, &GPS.nmea_latitude, &GPS.ns, &GPS.nmea_longitude,
				&GPS.ew, &GPS.fix, &GPS.num_of_satelites, &GPS.horizontal_dilution_of_precision,
				&GPS.mean_sea_level_altitude, &GPS.altitude_unit) >= 1)
		{
			GPS.dec_latitude = GPS_nmea_to_dec(GPS.nmea_latitude, GPS.ns);
			GPS.dec_longitude = GPS_nmea_to_dec(GPS.nmea_longitude, GPS.ew);
			return;
		}
	}
	else if (!strncmp(GPSstrParse, "$GPRMC", 6))
	{
		if (sscanf(GPSstrParse, "$GPRMC,%f,%f,%c,%f,%c,%f,%f,%d", &GPS.utc_time,
				&GPS.nmea_latitude, &GPS.ns, &GPS.nmea_longitude, &GPS.ew,
				&GPS.speed_over_ground, &GPS.course_over_ground, &GPS.utc_date) >= 1)
		{
			if (GPS.ns == '\0')
				GPS.ns = ' ';
			if (GPS.ew == '\0')
				GPS.ew = ' ';
			return;
		}

	}
	else if (!strncmp(GPSstrParse, "$GPGLL", 6))
	{
		if (sscanf(GPSstrParse, "$GPGLL,%f,%c,%f,%c,%f,%c", &GPS.nmea_latitude,
				&GPS.ns, &GPS.nmea_longitude, &GPS.ew, &GPS.utc_time,
				&GPS.gll_status) >= 1)
		{
			if (GPS.ns == '\0')
				GPS.ns = '?';
			if (GPS.ew == '\0')
				GPS.ew = '?';
			return;
		}
	}
	else if (!strncmp(GPSstrParse, "$GPVTG", 6))
	{
		if (sscanf(GPSstrParse, "$GPVTG,%f,%c,%f,%c,%f,%c,%f,%c", &GPS.course_t,
				&GPS.course_t_unit, &GPS.course_m, &GPS.course_m_unit,
				&GPS.speed_over_ground, &GPS.speed_k_unit, &GPS.speed_km,
				&GPS.speed_km_unit) >= 1)
		{
			if (GPS.ns == '\0')
				GPS.ns = '?';
			if (GPS.ew == '\0')
				GPS.ew = '?';
			return;
		}
	}
}

float GPS_nmea_to_dec(float deg_coord, char nsew)
{
	int degree = (int) (deg_coord / 100);
	float minutes = deg_coord - degree * 100;
	float dec_deg = minutes / 60;
	float decimal = degree + dec_deg;
	if (nsew == 'S' || nsew == 'W')
	{ // return negative
		decimal *= -1;
	}
	return decimal;
}
