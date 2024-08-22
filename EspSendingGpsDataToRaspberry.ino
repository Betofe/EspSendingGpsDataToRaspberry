/*
 Name:		EspSendingGpsDataToRaspberry.ino
 Created:	7/4/2023 2:01:14 PM
 Author:	Imami Joel Betofe
*/
#include "SendToRaspberry.h"
#include "MavlinkSettings.h"
#include "Globals.h"

MavlinkConnection Mavlink;
Sender sender;

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(9600);
	Mavlink.init();
	DEBUG_PORT.println("Sender Initiated");
	//sender.init();
}

// the loop function runs over and over again until power down or reset
void loop() {
	Mavlink.run();
}
