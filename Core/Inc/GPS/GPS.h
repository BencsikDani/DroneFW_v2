#ifndef GPS_H
#define GPS_H


typedef struct
{

	// calculated values
	float dec_longitude;
	float dec_latitude;
	float altitude_ft;

	// GGA - Global Positioning System Fixed Data
	float utc_time;
	float nmea_latitude;
	char ns;
	float nmea_longitude;
	char ew;
	int fix;	// 0 = Invalid
				// 1 = GNSS fix (SPS)
				// 2 = DGPS fix
				// 3 = PPS fix
				// 4 = Real Time Kinematic
				// 5 = Float RTK
				// 6 = estimated (dead reckoning) (2.3 feature)
				// 7 = Manual input mode
				// 8 = Simulation mode
	int num_of_satelites;
	float horizontal_dilution_of_precision;
	float mean_sea_level_altitude;
	char altitude_unit;

	// RMC - Recommended Minimmum Specific GNS Data
	char rmc_status;
	float speed_over_ground;
	float course_over_ground;
	int utc_date;

	// GLL
	char gll_status;	// A = Valud
						// V = Invalid

	// VTG - Course over ground, ground speed
	float course_t; // ground speed true
	char course_t_unit;
	float course_m; // magnetic
	char course_m_unit;
	char speed_k_unit;
	float speed_km; // speek km/hr
	char speed_km_unit;
} GPS_t;


uint8_t GPS_Init();
void GSP_USBPrint(char *data);
void GPS_print_val(char *data, int value);
void GPS_UART_CallBack();
int GPS_validate(char *nmeastr);
void GPS_parse(char *GPSstrParse);
float GPS_nmea_to_dec(float deg_coord, char nsew);

#endif /* GPS_H */
