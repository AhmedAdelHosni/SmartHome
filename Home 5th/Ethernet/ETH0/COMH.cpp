/******************************************************************************/
/*                Include common and project definition header                */
/******************************************************************************/
#include "Defined_Types.h"
#include "prj_pdf.h"

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/

#include "COMH.h"

/******************************************************************************/
/*                            Include other headers                           */
/******************************************************************************/

#ifdef ETHERNET_CONN
    #include <Ethernet.h>
    #include <MQTT.h>
#endif

/******************************************************************************/
/*                   Definition of local symbolic constants                   */
/******************************************************************************/


/******************************************************************************/
/*                  Definition of local function like macros                  */
/******************************************************************************/

#define BeginEthernetCommunication()     Ethernet.begin(mac, ip)
#define BeginClientCommunication()       client.begin(CLIENT_ADDR, net)
#define DefineClientCallback()           client.onMessage(mqtt_recieved_buffer)
#define ConnectToClient()                client.connect("arduino", "try", "try")
#define SubscribeToMqttTopic(x)          client.subscribe(x)

/******************************************************************************/
/*          Definition of local types (typedef, enum, struct, union)          */
/******************************************************************************/

/******************************************************************************/
/*                       Definition of local variables                        */
/******************************************************************************/

byte mac[] = MAC_ADDR;
byte ip[]  = IP_ADDR;

static char mqtt_topic_subscribe[MAX_MQTT_SUBSRIBE_SIZE] = MQTT_TOPIC_SUBSCRIBE_NAME;

static String mqtt_read_topic_buffer;
static String mqtt_read_payload_buffer;

u32 set_led_state_bitfields;

/******************************************************************************/
/*                  Declaration of local function prototypes                  */
/******************************************************************************/

void StartEthernetConnection(void) ;

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

#ifdef ETHERNET_CONN

void mqtt_recieved_buffer(String &topic, String &payload) {
    mqtt_read_topic_buffer = topic;
    mqtt_read_payload_buffer = payload;
}

void StartEthernetConnection(void) 
{
    DEBUG_SERIAL("connecting");

    while (!ConnectToClient()) 
    {
        DEBUG_SERIAL(".");
        DELAY_MS(1000);
    }
    
    DEBUG_SERIAL_NL("\nconnected!");
}

void COMH_INIT()
{
    BeginEthernetCommunication();
    BeginClientCommunication(); 
    DefineClientCallback();
    StartEthernetConnection();
    SubscribeToMqttTopic(mqtt_topic_subscribe);
}

void COMH_PublishMQTT(const String topic, const String payload)
{
    client.publish(topic, payload);
}



#endif
