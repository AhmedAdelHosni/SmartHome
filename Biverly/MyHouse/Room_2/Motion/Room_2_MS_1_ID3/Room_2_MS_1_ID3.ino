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
#define MY_NODE_ID 3 // Room_2_MS_1_ID3.ino

// Enable and select radio type attached
#define MY_RADIO_NRF24

// Enable repeater functionality for this node
#define MY_REPEATER_FEATURE

#include <MySensors.h>

#define DIGITAL_INPUT_SENSOR A1   // The digital input you attached your motion sensor.  (Only 2 and 3 generates interrupt!)
#define CHILD_ID 1   // Id of the sensor child

uint32_t SLEEP_TIME = 120000; // Sleep time between reports (in milliseconds)
static uint32_t sleepEnterMS = hwMillis();
static uint8_t first_run = 1;
static bool motion_state = 0;
static bool last_motion_state = 0;

// Initialize motion message
MyMessage msg(CHILD_ID, V_TRIPPED);

void setup()
{
  pinMode(DIGITAL_INPUT_SENSOR, INPUT);      // sets the motion sensor digital pin as input
}

void presentation()
{
  // Send the sketch version information to the gateway and Controller
    sendSketchInfo("Room_2_MS_1_ID3", "1.0");
  
    // Register all sensors to gw (they will be created as child devices)
    present(CHILD_ID, S_MOTION);
}

void loop()
{
  if(first_run == 0)
  { 
    motion_state = ((digitalRead(DIGITAL_INPUT_SENSOR)) == HIGH ? 1 : 0);
    
    if( ( motion_state != last_motion_state) || ((hwMillis() - sleepEnterMS) >= SLEEP_TIME ))
    {
      sleepEnterMS = hwMillis();
      send(msg.set(motion_state?"1":"0"));  // Send tripped value to gw
      last_motion_state = motion_state;
  //    Serial.println((bool) last_motion_state);
    }
  }
  else // first run. Wait for 500 ms and do not report anything. To avoid power loss and returing back which may result in send misleading states.
  {
    wait(500);
    motion_state = ((digitalRead(DIGITAL_INPUT_SENSOR)) == HIGH ? 1 : 0);
 //   Serial.print("Motion : ");
 //   Serial.println((bool) motion_state);
    first_run = 0;
  }
}



