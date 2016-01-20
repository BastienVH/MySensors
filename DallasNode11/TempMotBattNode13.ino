/**
* The MySensors Arduino library handles the wireless radio link and protocol
* between your home built sensors/actuators and HA controller of choice.
* The sensors forms a self healing radio network with optional repeaters. Each
* repeater and gateway builds a routing tables in EEPROM which keeps track of the
* network topology allowing messages to be routed to nodes.
*
* Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
* Copyright (C) 2013-2015 Sensnology AB
* Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
*
* Documentation: http://www.mysensors.org
* Support Forum: http://forum.mysensors.org
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
*******************************
*
* DESCRIPTION
*
* Example sketch showing how to send in DS1820B OneWire temperature readings back to the controller
* http://www.mysensors.org/build/temp
*/

// Include libraries
#include <MySensor.h>
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>

// Define data
#define CHILD_ID_TEMP 0              // Temp child ID
#define CHILD_ID_MOTION 2            // MOTION child ID
#define COMPARE_TEMP 1 // Send temperature only if changed? 1 = Yes 0 = No
#define ONE_WIRE_BUS 4 // Pin where dallase sensor is connected 
#define MAX_ATTACHED_DS18B20 16
#define MOTION_INPUT_SENSOR 3   // The digital input you attached your motion sensor.  (Only 2 and 3 generates interrupt!)
#define INTERRUPT MOTION_INPUT_SENSOR-2 // Usually the interrupt = pin -2 (on uno/nano anyway)

// Initialize mysensor
MySensor gw;

// Initialize D_Temp
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature.

									 // Variables
unsigned long SLEEP_TIME = 1200000; // Sleep time between reads (in milliseconds)
float lastTemperature[MAX_ATTACHED_DS18B20];
int numSensors = 0;
int node_id = 13;
int BATTERY_SENSE_PIN = A0;  // select the input pin for the battery sense point
int oldBatteryPcnt = 0;
boolean receivedConfig = false;
boolean metric = true;

// Initialize childs
MyMessage msgMot(CHILD_ID_MOTION, V_TRIPPED);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);

void setup()
{
	// Start up the OneWire library
	sensors.begin();
	// requestTemperatures() will not block current thread
	sensors.setWaitForConversion(false);

	// use the 1.1 V internal reference
#if defined(__AVR_ATmega2560__)
	analogReference(INTERNAL1V1);
#else
	analogReference(INTERNAL);
#endif

	// Startup and initialize MySensors library. Set callback for incoming messages.
	gw.begin(NULL, node_id);

	// Send the sketch version information to the gateway and Controller
	gw.sendSketchInfo("Temp en Motion sensor met batterij", "1.0");

	// Fetch the number of attached temperature sensors
	numSensors = sensors.getDeviceCount();

	// Present all sensors to controller
	for (int i = 0; i < numSensors && i < MAX_ATTACHED_DS18B20; i++) {
		gw.present(i, S_TEMP);
	}
	pinMode(MOTION_INPUT_SENSOR, INPUT);      // sets the motion sensor digital pin as input
											  // Register all sensors to gw (they will be created as child devices)
	gw.present(CHILD_ID_MOTION, S_MOTION);

}


void loop()
{
	int wake;
	wake = gw.sleep(INTERRUPT, CHANGE, SLEEP_TIME);
	if (wake == 1) {
		// Read digital motion value
		boolean tripped = digitalRead(MOTION_INPUT_SENSOR) == HIGH;

		Serial.println(tripped);
		gw.send(msgMot.set(tripped ? "1" : "0"));  // Send tripped value to gw 
	}
	else {
	// get the battery Voltage
	int sensorValue = analogRead(BATTERY_SENSE_PIN);
#ifdef DEBUG
	Serial.println(sensorValue);
#endif

	// 1M, 470K divider across battery and using internal ADC ref of 1.1V
	// Sense point is bypassed with 0.1 uF cap to reduce noise at that point
	// ((1e6+470e3)/470e3)*1.1 = Vmax = 3.44 Volts
	// 3.44/1023 = Volts per bit = 0.003363075
	float batteryV = sensorValue * 0.003363075;
	int batteryPcnt = sensorValue / 10;

#ifdef DEBUG
	Serial.print("Battery Voltage: ");
	Serial.print(batteryV);
	Serial.println(" V");

	Serial.print("Battery percent: ");
	Serial.print(batteryPcnt);
	Serial.println(" %");
#endif

	if (oldBatteryPcnt != batteryPcnt) {
		// Power up radio after sleep
		gw.sendBatteryLevel(batteryPcnt);
		oldBatteryPcnt = batteryPcnt;
	}

	// Process incoming messages (like config from server)
	gw.process();

	// Fetch temperatures from Dallas sensors
	sensors.requestTemperatures();

	// query conversion time and sleep until conversion completed
	int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());
	// sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
	gw.sleep(conversionTime);

	// Read temperatures and send them to controller
	for (int i = 0; i < numSensors && i < MAX_ATTACHED_DS18B20; i++) {

		// Fetch and round temperature to one decimal
		float temperature = static_cast<float>(static_cast<int>((gw.getConfig().isMetric ? sensors.getTempCByIndex(i) : sensors.getTempFByIndex(i)) * 10.)) / 10.;

		// Only send data if temperature has changed and no error
#if COMPARE_TEMP == 1
		if (lastTemperature[i] != temperature && temperature != -127.00 && temperature != 85.00) {
#else
		if (temperature != -127.00 && temperature != 85.00) {
#endif

			// Send in the new temperature
			gw.send(msgTemp.setSensor(i).set(temperature, 1));
			// Save new temperatures for next compare
			lastTemperature[i] = temperature;
		}
	}
	}
	gw.sleep(INTERRUPT, CHANGE, SLEEP_TIME);
}
