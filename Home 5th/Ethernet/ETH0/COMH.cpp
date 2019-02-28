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

static u32 set_led_state_bitfields = 0;
static u32 update_current_led_state = 0;

/******************************************************************************/
/*                  Declaration of local function prototypes                  */
/******************************************************************************/

void StartEthernetConnection(void) ;
void UpdateLedStates(u8 relay_index, u8 relay_new_state);

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

void mqtt_recieved_buffer(String &topic, String &payload) {
    String relay_index_str;    
    u8 relay_index_u8;
    u8 relay_index_new_state_u8;

    relay_index_str = topic.substring(8, payload.lastIndexOf("/"));
    relay_index_u8 = relay_index_str.toInt();

    relay_index_new_state_u8 = mqtt_read_payload_buffer.toInt();

    UpdateLedStates(relay_index_u8, relay_index_new_state_u8);
}

void StartEthernetConnection(void) 
{
    DEBUG_SERIAL("Connecting");

    while (!ConnectToClient()) 
    {
        DEBUG_SERIAL(".");
        DELAY_MS(1000);
    }
    
    DEBUG_SERIAL_NL("\nconnected!");
}

void COMH_INIT(void)
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

void UpdateLedStates(u8 relay_index, u8 relay_new_state)
{
    if(relay_new_state == 1)
    {
        set_led_state_bitfields |= (u32)((u32)1<<(u32) relay_index);
    }
    else if(relay_new_state == 0)
    {
        set_led_state_bitfields &= ~(1 <<(u32) relay_index);
    }
        
    update_current_led_state |= (u32)((u32)1<<(u32) relay_index);
}

u32 COMH_GetRequestedLedStates(void)
{
    return set_led_state_bitfields;
}

u32 COMH_GetConfirmedLedStates(void)
{
    return update_current_led_state;
}