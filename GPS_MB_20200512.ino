/*
 Name:		GPS_MB.ino
 Created:	13/02/2020 17:03:25
 Author:	Fernando

Requisitos/librerias: 	ArduinoModbusSlave-Master
						NeoGPS
						AltSoftSerial

15/05 Esta Version configura el esclavo modbus 1, a 9600,8,N,1
		el mapa modbus está publicado en:"\\server-pc\Diseño y Desarrollo\Periféricos\GPS\01_Documentacion Generada"
14/05 Acomodo date y time a Argentina -3 zone hour. 
	  la versión original pedia "day" en lugar de "date" y daba cualquier cosa (day es day of week!)
12/05 GuGa modifica (cuando no) Agrego registro de FIX en el mapa modbus, para saber cuando las cosas
      son válidas o no
	  Los pines del software serial con los que se comunica con el GPS son 8:rx y 9:tx
*/

//#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
//#include <ArduinoModbus.h>
#include "ModbusSlave.h"

#include "NMEAGPS.h"

#include "AltSoftSerial.h"
AltSoftSerial gpsPort; // 8:rx & 9:tx for an UNO or Pro Mini
#define GPS_PORT_NAME "AltSoftSerial"

//modbus_mapping_t modbusInfo;
Modbus slave(1, 7);
static NMEAGPS  gps;
static gps_fix  fix;

// Set these values to the offset of your timezone from GMT

static const int32_t          zone_hours = -3L; // EST
static const int32_t          zone_minutes = 0L; // usually zero
static const NeoGPS::clock_t  zone_offset = zone_hours * NeoGPS::SECONDS_PER_HOUR + zone_minutes * NeoGPS::SECONDS_PER_MINUTE;

float flat, flon, alt;
unsigned long age;
uint16_t year;
//uint32_t* a;
uint16_t month, day, hour, minute, second, hundredths;
uint16_t fix_mode; 
// los valores que tendré en fix_mode_low los retraigo del status 
// VALID de cada uno. 
// Bit 0: location, 1:altitude, 2: date &time juntos
static void callMODBUS(void);
static void GPSloop(void);

void setup()
{
	gpsPort.begin(9600);

	/* register handler functions
	 * into the modbus slave callback vector.
	 */
	slave.cbVector[CB_READ_INPUT_REGISTERS] = readGPSData;
	// set Serial and slave at baud 9600.
	Serial.begin(9600);
	slave.begin(9600);
}

void loop()
{
	GPSloop();
	slave.poll();
}

static void GPSloop()
{
	while (gps.available(gpsPort)) {
		fix = gps.read();
		callMODBUS();
	}
} // GPSloop

void adjustTime( NeoGPS::time_t & dt )
{
  NeoGPS::clock_t seconds = dt; // convert date/time structure to seconds
  //  First, offset from UTC to the local timezone
  seconds += zone_offset;
  dt = seconds; // convert seconds back to a date/time structure
} // adjustTime

static void callMODBUS(void)
{
	if (fix.valid.location) {
		flat = fix.latitude();
		flon = fix.longitude();
		bitSet(fix_mode,0);		//VALID
	}
 	else
 	{
		bitClear(fix_mode,0);		//not valid
  	}
	if (fix.valid.altitude) {
		alt = fix.altitude();
		bitSet(fix_mode,1);
	}
	else
	{
		bitClear(fix_mode,1);
	}
	

	if (fix.valid.time && fix.valid.date) 
	{
		adjustTime( gps.fix().dateTime );	//acomodo a Argentina
		hour =   gps.fix().dateTime.hours;
		minute = gps.fix().dateTime.minutes;
		second = gps.fix().dateTime.seconds;
		day =   gps.fix().dateTime.date;
		month = gps.fix().dateTime.month;
		year =  gps.fix().dateTime.year;		

// Values without adjust
		// day = gps.fix().dateTime.date;
		// month = gps.fix().dateTime.month;
		// year = gps.fix().dateTime.year;
		// hour = gps.fix().dateTime.hours;
		// minute = gps.fix().dateTime.minutes;
		// second = gps.fix().dateTime.seconds;

		bitSet(fix_mode,2);
	}
	else
	{
		bitClear(fix_mode,2);
	}
	
} // callMODBUS

/**
 * Handle Read Input Registers (FC=04)
 * write back the values from analog in pins (input registers).
 */
uint8_t readGPSData(uint8_t fc, uint16_t address, uint16_t length) {

	slave.writeRegisterToBuffer(0, (uint16_t)((*((uint32_t*)&flat) & 0xFFFF0000) >> 16) );
	slave.writeRegisterToBuffer(1, (uint16_t)(*((uint32_t*)&flat) & 0x0000FFFF) );

	slave.writeRegisterToBuffer(2, (uint16_t)((*((uint32_t*)&flon) & 0xFFFF0000) >> 16) );
	slave.writeRegisterToBuffer(3, (uint16_t)(*((uint32_t*)&flon) & 0x0000FFFF) );

	slave.writeRegisterToBuffer(4, (uint16_t)((*((uint32_t*)&alt) & 0xFFFF0000) >> 16) );
	slave.writeRegisterToBuffer(5, (uint16_t)(*((uint32_t*)&alt) & 0x0000FFFF) );

	slave.writeRegisterToBuffer(6, hour);
	slave.writeRegisterToBuffer(7, minute);
	slave.writeRegisterToBuffer(8, second);

	slave.writeRegisterToBuffer(9, day);
	slave.writeRegisterToBuffer(10, month);
	slave.writeRegisterToBuffer(11, year);

  slave.writeRegisterToBuffer(12, fix_mode); //0 not valid, 

	return STATUS_OK;
} // readAnalogIn
