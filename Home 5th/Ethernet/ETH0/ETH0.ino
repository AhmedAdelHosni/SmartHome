
/******************************************************************************/
/*                Include common and project definition header                */
/******************************************************************************/

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/


/******************************************************************************/
/*                            Include other headers                           */
/******************************************************************************/
#include <EEPROM.h>
#include <Ethernet.h>
#include <MQTT.h>

/******************************************************************************/
/*                   Definition of local symbolic constants                   */
/******************************************************************************/

#define DEVICE_NAME                 "ETH0"
#define INTERVAL_PERIOD_50_MS       50

/******************************************************************************/
/*                  Definition of local function like macros                  */
/******************************************************************************/

/******************************************************************************/
/*          Definition of local types (typedef, enum, struct, union)          */
/******************************************************************************/

enum PIN_x
{
  UNUSED_PIN_X_0,
  PIN_A,
  PIN_B,
  PIN_C,
  PIN_D,
  PIN_E,
  PIN_F,
  PIN_G,
  PIN_H,
  UNUSED_PIN_X_1,
  PIN_J,
  PIN_K,
  PIN_L
} PIN_x_E;

enum sensor_type
{
	CONTACT = 0,
	STATUS
}

/******************************************************************************/
/*                       Definition of local variables                        */
/******************************************************************************/

static u8 pin_d_states_previous = 0;
static u8 pin_c_states_previous = 0;
static u8 pin_l_states_previous = 0;
static u8 pin_f_states_previous = 0;
static u8 pin_k_states_previous = 0;

static unsigned long current_millis = millis();
static unsigned long previous_millis_50ms = 0;

static char motion_sensor_mqtt_topics[50][5] = {};
static char topic_name_mqtt [50];
static char topic_value_mqtt[50];
static char mqtt_topic      [50];

static String topic_value = "";

static u8 input_pin_number[] = { 22, 23, 24, 25, 26, 27 ,28, 29,
								 37, 36, 35, 34, 33, 32, 31, 30,
								 49, 48, 47, 46, 45, 44, 43, 42,
								 54, 55, 56, 57, 58, 59, 60, 61,
								 62, 63, 64, 65, 66, 67, 68, 69
								};

static String input_sensor_type[] = { "D", "D", "D", "D", "D", "D" ,"D", "D",
                                      "D", "W", "W", "W", "W", "W", "W", "W",
                                      "W", "W", "M", "M", "M", "M", "M", "M",
									  "L", "L", "L", "L", "L", "L", "L", "L",
									  "L", "L", "L", "L", "L", "L", "L", "L"
                                    };



byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
byte ip[] = {192, 168, 100, 177};  // <- change to match your network

/******************************************************************************/
/*                  Declaration of local function prototypes                  */
/******************************************************************************/

void process_inputs(void);
void process_outputs(void);

/******************************************************************************/
/*                        Declaration of local Objects                        */
/******************************************************************************/
EthernetClient net;
MQTTClient client;

/******************************************************************************/
/*                       Definition of local functions                        */
/******************************************************************************/

/*
void output_high(u8 * port_name, m_sensor_E pin)
{
    if (pin < 8)
    {
        *port_name |= (u8)((u8)1<<(u8) pin);
    }
}
*/
/*
void output_low(u8 * port_name, u8 pin)
{
    if (pin < 8)
    {
        *port_name &= ~(1 <<(u8) pin);
    }
}
*/

void connect() {
  Serial.print("connecting...");
  while (!client.connect("arduino", "try", "try")) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nconnected!");
}




void setup()
{

#ifdef ARDUINO_AVR_UNO
  PORTD = 0xFC;    //Set port D pin 7 6 5 4 3 2 as pull up
#elif ARDUINO_AVR_MEGA2560
  PORTA = 0xFF;    //Set port A pin 7 6 5 4 3 2 1 0 as pull up for MEGA
  PORTC = 0xFF;    //Set port C pin 7 6 5 4 3 2 1 0 as pull up for MEGA
  PORTL = 0xFF;    //Set port L pin 7 6 5 4 3 2 1 0 as pull up for MEGA
  PORTF = 0xFF;    //Set port A pin 7 6 5 4 3 2 1 0 as pull up for MEGA (( ANALOUGE)
  PORTK = 0xFF;    //Set port A pin 7 6 5 4 3 2 1 0 as pull up for MEGA (( ANALOUGE)
#endif

  Serial.begin(115200);
  Ethernet.begin(mac, ip);

  // Note: Local domain names (e.g. "Computer.local" on OSX) are not supported by Arduino.
  // You need to set the IP address directly.
  client.begin("192.168.100.107", net);
  client.onMessage(messageReceived);

  connect();
  client.subscribe(mqtt_topic);


  Serial.println("Begin");
  Serial.println(motion_sensor_mqtt_topics[0]);
  char * words = "hamda";
  strcpy(motion_sensor_mqtt_topics[33], words);

  Serial.println(motion_sensor_mqtt_topics[33]);
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
}



void publish_updated_pins(u8 * pin_states, u8 * pin_states_previous, u8 pin_start, enum sensor_type sensor_type_e)
{
  if(( (*pin_states) ^ (*pin_states_previous) ) != 0)
  {
    u8 current_pin_states = *pin_states;
    u8 updated_pins       = current_pin_states ^ *pin_states_previous;

    for (u8 pin_i = 0; pin_i <= 7; pin_i++)
    {
      if ( (((u8)(updated_pins >> (u8) pin_i)) & (u8)1) != (u8)0 )
      {
        String topic_name = "/" + String(DEVICE_NAME) + "/" + input_sensor_type[pin_i + pin_start] + "/" + String(input_pin_number[pin_i + pin_start]) + "/";
        topic_name.toCharArray(topic_name_mqtt, topic_name.length()+1);

        if(sensor_type_e == CONTACT) // For Contact switches like Doors, Windows & Motion
		{
			((current_pin_states >> pin_i) & 1) ? (topic_value = "OPEN") : (topic_value = "CLOSED");
		}
		else if(sensor_type_e == STATUS) // For AC status report feedback to OpenHAB. 
		{
			((current_pin_states >> pin_i) & 1) ? (topic_value = "ON") : (topic_value = "OFF");
		}
		
        topic_value.toCharArray(topic_value_mqtt, topic_value.length()+1);

        client.publish(topic_name_mqtt , topic_value_mqtt);
      }
    }
    *pin_states_previous = current_pin_states;
  }
}

void process_inputs(void)
{
	publish_updated_pins(port_to_input_PGM[PIN_A], &pin_d_states_previous, 0,  CONTACT);
    publish_updated_pins(port_to_input_PGM[PIN_C], &pin_c_states_previous, 8,  CONTACT);
    publish_updated_pins(port_to_input_PGM[PIN_L], &pin_l_states_previous, 16, CONTACT);
    publish_updated_pins(port_to_input_PGM[PIN_F], &pin_f_states_previous, 24, CONTACT);
    publish_updated_pins(port_to_input_PGM[PIN_K], &pin_k_states_previous, 32, CONTACT);
}

void process_outputs(void)
{
	
}

void loop(void)
{
  current_millis = millis();

  client.loop();

  if (!client.connected()) {
    connect();
  }

  if ((unsigned long)(current_millis - previous_millis_50ms) >= INTERVAL_PERIOD_50_MS)  // 50ms cyclic
  {
    previous_millis_50ms = millis();
    
	process_inputs();
	process_outputs();
  }
}