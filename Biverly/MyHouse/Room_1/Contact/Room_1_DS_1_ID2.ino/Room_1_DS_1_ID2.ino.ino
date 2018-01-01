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
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 *
 * DESCRIPTION
 * Motion Sensor example using HC-SR501
 * http://www.mysensors.org/build/motion
 *
 */

// Enable debug prints
#define MY_DEBUG

// base id for this house ( shall be unique for each house )
#define MY_RF24_BASE_RADIO_ID 0xd1,0x24,0xdd,0xde,0x99

// Static node id
#define MY_NODE_ID 2 // Room_1_DS_1_ID2.ino

// Enable and select radio type attached
#define MY_RADIO_NRF24

#include <MySensors.h>

#define DIGITAL_INPUT_SENSOR 3   // The digital input you attached your motion sensor.  (Only 2 and 3 generates interrupt!)
#define CHILD_ID 1   // Id of the sensor child

#define SLEEP_TIME 21600000 // report every 6 hrs
static uint32_t sleepEnterMS = hwMillis();
static uint8_t first_run = 1;
static bool motion_state = 0;
static bool last_motion_state = 0;


int BATTERY_SENSE_PIN = A0;  // select the input pin for the battery sense point
int oldBatteryPcnt = 0;


// Initialize motion message
MyMessage msg(CHILD_ID, V_TRIPPED);

void setup()
{
// use the 1.1 V internal reference
#if defined(__AVR_ATmega2560__)
    analogReference(INTERNAL1V1);
#else
    analogReference(INTERNAL);
#endif

	pinMode(DIGITAL_INPUT_SENSOR, INPUT);      // sets the motion sensor digital pin as input
}

void presentation()
{
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo("Room_1_DS_1_ID2", "1.1 2017.12.31");

	// Register all sensors to gw (they will be created as child devices)
	present(CHILD_ID, S_MOTION);
}

void loop()
{
    int sensorValue = analogRead(BATTERY_SENSE_PIN);

    int batteryPcnt = sensorValue / 10;

    float batteryV  = sensorValue * 0.003363075;
    
    sendBatteryLevel(batteryPcnt);
    oldBatteryPcnt = batteryPcnt;
        
    motion_state = ((digitalRead(DIGITAL_INPUT_SENSOR)) == HIGH ? 1 : 0);
    
    if( ( motion_state != last_motion_state) )
    {
      send(msg.set(motion_state?"1":"0"));  // Send tripped value to gw
      last_motion_state = motion_state;
    }
  
	  sleep(digitalPinToInterrupt(DIGITAL_INPUT_SENSOR), CHANGE, 0);
}


