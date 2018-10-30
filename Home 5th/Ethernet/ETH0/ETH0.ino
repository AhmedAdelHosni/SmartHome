

#include <Ethernet.h>
#include <MQTT.h>


#define DEVICE_NAME "ETH0"
#define INTERVAL_PERIOD_50_MS 50

//typedef unsigned int  u8;

static unsigned long current_millis = millis();
static unsigned long previous_millis_50ms = 0;
  
static u8* pin_d_states = &PINK;
static u8 pin_d_states_previous = 0;

static String topic_value = "";

char topic_name_mqtt[50];
char topic_value_mqtt[50];

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
byte ip[] = {192, 168, 100, 177};  // <- change to match your network

EthernetClient net;
MQTTClient client;

typedef enum
{
    LAUNDRY,
    BATHROOM,
    ENTRANCE = 2,
    MASTER_ROOM,
    KITCHECN,
    LIVING,
    BOYS,
    GIRLS,
    MOTION_MAX_NUM
} m_sensor_E;

char motion_sensor_mqtt_topics[MOTION_MAX_NUM][15] = 
{ 
    "LAUNDRY",
    "BATHROOM",
    "ENTRANCE",
    "MASTER_ROOM",
    "KITCHECN",
    "LIVING",
    "BOYS",
    "GIRLS"
};


void output_high(u8 * port_name, m_sensor_E pin)
{
    if (pin < 8)
    {
        *port_name |= (u8)((u8)1<<(u8) pin);
    }
}

void output_low(u8 * port_name, m_sensor_E pin)
{
    if (pin < 8)
    {
        *port_name &= ~(1 <<(u8) pin);
    }
}


void connect() {
  Serial.print("connecting...");
  while (!client.connect("arduino", "try", "try")) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");

  client.subscribe("/hello");
  // client.unsubscribe("/hello");
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
}



void setup() 
{

#ifdef AVR_UNO
  PORTD = 0xFC;    //Set port D pin 7 6 5 4 3 2 as pull up
#elif AVR_MEGA

#endif
  PORTA = 0xFF;    //Set port A pin 7 6 5 4 3 2 1 0 as pull up for MEGA
  PORTC = 0xFF;    //Set port C pin 7 6 5 4 3 2 1 0 as pull up for MEGA
  PORTL = 0xFF;    //Set port L pin 7 6 5 4 3 2 1 0 as pull up for MEGA
 // PORTF = 0xFF;    //Set port A pin 7 6 5 4 3 2 1 0 as pull up for MEGA (( ANALOUGE)
  PORTK = 0xFF;    //Set port A pin 7 6 5 4 3 2 1 0 as pull up for MEGA (( ANALOUGE)
  
  Serial.begin(115200);
  Ethernet.begin(mac, ip);

  // Note: Local domain names (e.g. "Computer.local" on OSX) are not supported by Arduino.
  // You need to set the IP address directly.
  client.begin("192.168.100.107", net);
  client.onMessage(messageReceived);

  connect();

  
  Serial.println("Begin");
}


#include <stdio.h>

void loop()
{
     current_millis = millis();

  client.loop();

  if (!client.connected()) {
    connect();
  }
  
  if ((unsigned long)(current_millis - previous_millis_50ms) >= INTERVAL_PERIOD_50_MS)  // 50ms cyclic
  {
     previous_millis_50ms = millis();    

    if(( (*pin_d_states) ^ (pin_d_states_previous) ) != 0)
    {
      u8 current_pin_states = *pin_d_states;
      u8 updated_pins = current_pin_states ^ pin_d_states_previous;
      
      for (u8 pin_i = 0; pin_i < MOTION_MAX_NUM; pin_i++)
      {
        if ( (((u8)(updated_pins >> (u8) pin_i)) & (u8)1) != (u8)0 )
        {
    //       Serial.print("/");
    //       Serial.print(DEVICE_NAME);
    //      Serial.print("/MOTION/");
        //  Serial.print(motion_sensor_mqtt_topics[pin_i]);
      //    Serial.print("/");
        //  Serial.println(((*pin_d_states >> pin_i) & 1));
    
          String topic_name = "/" + String(DEVICE_NAME) + "/MOTION/" + String(motion_sensor_mqtt_topics[pin_i]) + "/";
          topic_name.toCharArray(topic_name_mqtt, topic_name.length()+1);
    Serial.println(topic_name.length());
          ((current_pin_states >> pin_i) & 1) ? topic_value = "OPEN" : topic_value = "CLOSED";
          topic_value.toCharArray(topic_value_mqtt, topic_value.length()+1);
    
          client.publish(topic_name_mqtt , topic_value_mqtt);
        }
      }      
      pin_d_states_previous = current_pin_states;
    }
  }
}

