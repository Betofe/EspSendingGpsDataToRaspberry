// 
// 
// 

#include "SendToRaspberry.h"
#include "MavlinkSettings.h"
#include "Globals.h"
#include <HardwareSerial.h>

//MavlinkConnection Mavlink;

//HardwareSerial espSerial(1);
 

void Sender::init() {
	// Initialize the serial port at 115200 baud
	//Serial.begin(BAUD_RATE);
	// Initialize the software serial port at 9600 baud
	// espSerial.begin(9600, SERIAL_8N1, 18, 19);
}

void Sender::run() {
	
	Serial.print("LONG: ");
	Serial.println(gps_coordinates[LONG]);
	//delay(1000);
	Serial.print("LAT: ");
	Serial.println(gps_coordinates[LAT]);
	//delay(1000);
	Serial.print("ALT: ");
	Serial.println(gps_coordinates[ALT]);
	//delay(1000);
	Serial.print("RELALT: ");
	Serial.println(gps_coordinates[RELALT]);
	//espSerial.write((byte*)gps_coordinates, sizeof(gps_coordinates));
	//delay(1000);
	Serial.println("----------------------------");
}


