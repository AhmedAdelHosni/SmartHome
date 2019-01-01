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
#define TRUE  1
#define FALSE 0

#define ON  1
#definf OFF 0

#define ETHERNET_CONN
#define DEVICE_NAME                 "E0" // Ethernet node 0
#define INTERVAL_PERIOD_50_MS       50

#define OUTPUT_TOGGLE(port_name, pin) ()port_name ^= (1UL << pin));
/******************************************************************************/
/*                  Definition of local function like macros                  */
/******************************************************************************/

/******************************************************************************/
/*          Definition of local types (typedef, enum, struct, union)          */
/******************************************************************************/

enum PIN_x {
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

enum sensor_type {
  CONTACT = 0,
  STATUS
};

struct ac_parameters {
  bool curr_state;
};

/******************************************************************************/
/*                       Definition of local variables                        */
/******************************************************************************/

static u8 pin_d_states_previous = 0;
static u8 pin_c_states_previous = 0;
static u8 pin_l_states_previous = 0;
static u8 pin_f_states_previous = 0;
static u8 pin_k_states_previous = 0;

static u8 mqtt_is_new_packet_available = 0;

static unsigned long current_millis = millis();
static unsigned long previous_millis_50ms = 0;

static char mqtt_publish_topic_name  [50];
static char mqtt_publish_topic_value [50];
static char mqtt_topic_subscribe     [10] = "OH/E0/#";

static String mqtt_read_topic_buffer;
static String mqtt_read_payload_buffer;

static u8 input_pin_number[] = { 22, 23, 24, 25, 26, 27 ,28, 29,
                                 37, 36, 35, 34, 33, 32, 31, 30,
                                 49, 48, 47, 46, 45, 44, 43, 42,
                                 54, 55, 56, 57, 58, 59, 60, 61,
                                 62, 63, 64, 65, 66, 67, 68, 69
                                };

static u8 output_toggle_port[18] = {OUTPUT_TOGGLE(PORTE, 0), OUTPUT_TOGGLE(PORTE, 1), OUTPUT_TOGGLE(PORTE, 3), OUTPUT_TOGGLE(PORTE, 4), OUTPUT_TOGGLE(PORTE, 5), OUTPUT_TOGGLE(PORTG, 5),
                                    OUTPUT_TOGGLE(PORTH, 0), OUTPUT_TOGGLE(PORTH, 1), OUTPUT_TOGGLE(PORTH, 3), OUTPUT_TOGGLE(PORTH, 4), OUTPUT_TOGGLE(PORTH, 5), OUTPUT_TOGGLE(PORTH, 6),
						            OUTPUT_TOGGLE(PORTB, 5), OUTPUT_TOGGLE(PORTB, 6), OUTPUT_TOGGLE(PORTB, 7), OUTPUT_TOGGLE(PORTJ, 0), OUTPUT_TOGGLE(PORTJ, 1), OUTPUT_TOGGLE(PORTD, 2),
						            OUTPUT_TOGGLE(PORTD, 3)};
						   
static String input_sensor_type[] = { "D", "D", "D", "D", "D", "D" ,"D", "D",
                                      "D", "W", "W", "W", "W", "W", "W", "W",
                                      "W", "W", "M", "M", "M", "M", "M", "M",
                                      "L", "L", "L", "L", "L", "L", "L", "L",
                                      "L", "L", "L", "L", "L", "L", "L", "L"
                                    };

static struct ac_parameters ac_parameters_s[18] = {0};
//static struct ac_parameters ac_parameters_f_port[8] = {0};
//static struct ac_parameters ac_parameters_k_port[8] = {0};

#ifdef ETHERNET_CONN
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
byte ip[] = {192, 168, 100, 177};  // <- change to match your network
#endif
/******************************************************************************/
/*                  Declaration of local function prototypes                  */
/******************************************************************************/

void process_inputs(void);
void process_outputs(void);
void check_mqtt_error(void)
void TurnOffSwitch(u8 pin_n);
void TurnOnSwitch(u8 pin_n);
void publish_updated_pins(u8 * pin_states, u8 * pin_states_previous, u8 pin_start, enum sensor_type sensor_type_e);

/******************************************************************************/
/*                        Declaration of local Objects                        */
/******************************************************************************/

#ifdef ETHERNET_CONN
EthernetClient net;
MQTTClient client;
#endif

/******************************************************************************/
/*                       Definition of local functions                        */
/******************************************************************************/

void output_toggle(u8* port_name, u8 pin)
{
	port_name ^= (1UL << pin);
}
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
#ifdef ETHERNET_CONN
void connect() {
  Serial.print("connecting...");
  while (!client.connect("arduino", "try", "try")) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nconnected!");
}
#endif

#ifdef ETHERNET_CONN
void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
  mqtt_read_topic_buffer = topic;
  mqtt_read_payload_buffer_buffer = payload;
  mqtt_is_new_packet_available = mqtt_is_new_packet_available + 1;
}
#endif


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
  
  DDRE = 0b00111011; // Set port E bit 0,1,3,4,5 to OUTPUT
#endif

  Serial.begin(115200);
#ifdef ETHERNET_CONN
  Ethernet.begin(mac, ip);

  // Note: Local domain names (e.g. "Computer.local" on OSX) are not supported by Arduino.
  // You need to set the IP address directly.
  client.begin("192.168.100.106", net);
  client.onMessage(messageReceived);

  connect();
  client.subscribe(mqtt_topic_subscribe);


  client.publish("ETH0/state", "System was restared");
#endif
}
void publish_updated_pins(u8 * pin_states, u8 * pin_states_previous, u8 pin_start, enum sensor_type sensor_type_e)
{
  String topic_value = "NA";
  
  if(( (*pin_states) ^ (*pin_states_previous) ) != 0)
  {
    u8 current_pin_states = *pin_states;
    u8 updated_pins       = current_pin_states ^ *pin_states_previous;

    for (u8 pin_i = 0; pin_i <= 7; pin_i++)
    {
      if ( (((u8)(updated_pins >> (u8) pin_i)) & (u8)1) != (u8)0 )
      {
        String topic_name = "/" + String(DEVICE_NAME) + "/" + input_sensor_type[pin_i + pin_start] + "/" + String(input_pin_number[pin_i + pin_start]) + "/";
        topic_name.toCharArray(mqtt_publish_topic_name, topic_name.length()+1);

        if(sensor_type_e == CONTACT) // For Contact switches feedback like Doors, Windows & Motion
        {
          ((current_pin_states >> pin_i) & 1) ? (topic_value = "OPEN") : (topic_value = "CLOSED");
        }
        else if(sensor_type_e == STATUS) // For AC status report feedback to OpenHAB. 
        {
          ((current_pin_states >> pin_i) & 1) ? (topic_value = "ON") : (topic_value = "OFF");
		  ac_parameters_s.state[pin_start - 24 + pin_i] = ((current_pin_states >> pin_i) & 1); // put the current ac state in position pin_start - ( 24 which is first pin_start for ac state) + the pin_i
		/*
		  if(pin_start >= 32)
		  {
		    ac_parameters_k_port.state[pin_i] = ((current_pin_states >> pin_i) & 1);
		  }
		  else if(pin_start >= 24)
		  {
		    ac_parameters_f_port.state[pin_i] = ((current_pin_states >> pin_i) & 1);
		  }
        }
        */
        topic_value.toCharArray(mqtt_publish_topic_value, topic_value.length()+1);
 
        Serial.print(topic_name + " " );
        Serial.println(topic_value);
#ifdef ETHERNET_CONN
        client.publish(mqtt_publish_topic_name , mqtt_publish_topic_value);
        client.publish("testsw/1", "OFF");
#endif
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
    publish_updated_pins(port_to_input_PGM[PIN_F], &pin_f_states_previous, 24, STATUS);
    publish_updated_pins(port_to_input_PGM[PIN_K], &pin_k_states_previous, 32, STATUS);
}
void TurnOnSwitch(u8 pin_n)
{
  if(ac_parameters_s[pin_n] == FALSE)
  {
    output_toggle_port[pin_n];
  }
}

void TurnOffSwitch(u8 pin_n)
{
  if(ac_parameters_s[pin_n] == TRUE)
  {
    output_toggle_port[pin_n];
  }
}

void process_outputs(void)
{
  u8 pin_n = mqtt_read_topic_buffer; // Currently this is not correct. Shall extract the pin name from the buffer
  u8 payload = mqtt_read_payload_buffer_buffer;
  
  if(mqtt_is_new_packet_available != 0)
  {
    if(payload == ON)
	{
		TurnOnSwitch(pin_n);
	}
	else if(payload == OFF)
	{
		TurnOffSwitch(pin_n);
	}
	
    mqtt_is_new_packet_available = mqtt_is_new_packet_available - 1;
  }
}

void check_mqtt_error(void)
{
  if(mqtt_is_new_packet_available > 1)
  {
    Serial.println("ERR : MQTT PACKET MISSED. Counter is %d", mqtt_is_new_packet_available);
#ifdef ETHERNET_CONN	 
    client.publish("E0/OH/ERRMQTT", mqtt_is_new_packet_available);	  
#endif   
  }
}

void loop(void)
{
  current_millis = millis();

#ifdef ETHERNET_CONN
  client.loop();
  
  if (!client.connected()) {
    connect();
  }
#endif

  process_outputs();
  check_mqtt_error();
   
  if ((unsigned long)(current_millis - previous_millis_50ms) >= INTERVAL_PERIOD_50_MS)  // 50ms cyclic
  {
    previous_millis_50ms = millis();
    
    process_inputs();
  }
}