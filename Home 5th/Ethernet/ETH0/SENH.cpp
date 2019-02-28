/******************************************************************************/
/*                Include common and project definition header                */
/******************************************************************************/
#include "Defined_Types.h"
#include "prj_pdf.h"

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/

#include "SENH.h"

/******************************************************************************/
/*                            Include other headers                           */
/******************************************************************************/

#include "IoHw.h"
#include "COMH.h"

/******************************************************************************/
/*                   Definition of local symbolic constants                   */
/******************************************************************************/

#define PORT_D_7_INPUT_PIN_ARRAY_INDEX                  40
#define PORT_G_2_INPUT_PIN_ARRAY_INDEX                  41
#define PORT_G_1_INPUT_PIN_ARRAY_INDEX                  42
#define PORT_G_0_INPUT_PIN_ARRAY_INDEX                  43

#define PORT_D_PIN_7                                    7
#define PORT_G_PIN_2                                    2
#define PORT_G_PIN_1                                    1
#define PORT_G_PIN_0                                    0

/******************************************************************************/
/*                  Definition of local function like macros                  */
/******************************************************************************/

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

/******************************************************************************/
/*          Definition of local types (typedef, enum, struct, union)          */
/******************************************************************************/

enum sensor_type {
    CONTACT = 0,
    STATUS  = 1,
    NA      = 2
};

struct input_parameters {
    enum IOHW_port_pins_x port_name;
    u8 pin_previous_states;
    u8 pin_input_start;
    enum sensor_type sensor_type_e;
};

struct input_parameters_per_pin {
    enum IOHW_port_pins_x port_name;
    u8 pin_previous_states;
    u8 port_pin_x;
    u8 port_input_pin_array_index;
    enum sensor_type sensor_type_e;
};

/******************************************************************************/
/*                       Definition of local variables                        */
/******************************************************************************/

  // TODO : implement in SENH_Init() all initializations

static char mqtt_publish_topic_name  [50];
static char mqtt_publish_topic_value [50]; 
   

static String input_pin_number[]          = { "22", "23", "24", "25", "26", "27", "28", "29",
                                              "37", "36", "35", "34", "33", "32", "31", "30",
                                              "49", "48", "47", "46", "45", "44", "43", "42",
                                              "54", "55", "56", "57", "58", "59", "60", "61",
                                              "62", "63", "64", "65", "66", "67", "68", "69",
                                              "38", "39", "40", "41"
                                           };

static String input_sensor_type[]        = {  "D", "D", "D", "D", "D", "D" ,"D", "D",
                                              "D", "W", "W", "W", "W", "W", "W", "W",
                                              "W", "W", "M", "M", "M", "M", "M", "M",
                                              "L", "L", "L", "L", "L", "L", "L", "L",
                                              "L", "L", "L", "L", "L", "L", "L", "L",
                                              "D", "L", "L", "D"
                                           };

static u8 pin_state_status[18] = {0};

static struct input_parameters input_parameters_a = {PORT_A, 0, 0 , CONTACT};
static struct input_parameters input_parameters_c = {PORT_C, 0, 8 , CONTACT};
static struct input_parameters input_parameters_l = {PORT_L, 0, 16, CONTACT};
static struct input_parameters input_parameters_f = {PORT_F, 0, 24, STATUS };
static struct input_parameters input_parameters_k = {PORT_K, 0, 32, STATUS };

static struct input_parameters_per_pin input_parameters_d_7 = {PORT_D, 0, PORT_D_PIN_7, PORT_D_7_INPUT_PIN_ARRAY_INDEX, CONTACT};
static struct input_parameters_per_pin input_parameters_g_2 = {PORT_G, 0, PORT_G_PIN_2, PORT_G_2_INPUT_PIN_ARRAY_INDEX, STATUS };
static struct input_parameters_per_pin input_parameters_g_1 = {PORT_G, 0, PORT_G_PIN_1, PORT_G_1_INPUT_PIN_ARRAY_INDEX, STATUS };
static struct input_parameters_per_pin input_parameters_g_0 = {PORT_G, 0, PORT_G_PIN_0, PORT_G_0_INPUT_PIN_ARRAY_INDEX, CONTACT};

/******************************************************************************/
/*                  Declaration of local function prototypes                  */
/******************************************************************************/

void ProcessInputs(void);
void ProcessUpdatedPins(struct input_parameters* param);
void ProcessUpdatedPinsIndividually(struct input_parameters_per_pin* param);
void PublishUpdatedInputs(String input_sensor_type_param, String input_pin_number_param, String topic_value);

/******************************************************************************/
/*                        Declaration of local Objects                        */
/******************************************************************************/

/******************************************************************************/
/*                       Definition of local functions                        */
/******************************************************************************/

void SENH_Init(void)
{

}

void SENH_Cyclic(void)
{
    ProcessInputs();
}

void ProcessInputs(void)
{
    ProcessUpdatedPins(&input_parameters_a);
    ProcessUpdatedPins(&input_parameters_c);
    ProcessUpdatedPins(&input_parameters_l);
    ProcessUpdatedPins(&input_parameters_f);
    ProcessUpdatedPins(&input_parameters_k);

    ProcessUpdatedPinsIndividually(&input_parameters_d_7);
    ProcessUpdatedPinsIndividually(&input_parameters_g_2);
    ProcessUpdatedPinsIndividually(&input_parameters_g_1);
    ProcessUpdatedPinsIndividually(&input_parameters_g_0);
}

void ProcessUpdatedPinsIndividually(struct input_parameters_per_pin* param)
{
    String topic_value;
    u8 pin_current_states = *IOHW_GetPortPinStatus(param->port_name);

    if(param->pin_previous_states != CHECK_BIT(pin_current_states, param->port_pin_x))
    {
        u8 current_pin_states = pin_current_states;

        if(param->sensor_type_e == CONTACT)
        {
            (((current_pin_states >> param->port_pin_x) & 1) != 0) ? (topic_value = "OPEN") : (topic_value = "CLOSED");
        }
        else if(param->sensor_type_e == STATUS)
        {
            (((current_pin_states >> param->port_pin_x) & 1) != 0) ? (topic_value = "OFF") : (topic_value = "ON");
            pin_state_status[param->port_input_pin_array_index - 25] = ((current_pin_states >> param->port_pin_x) & 1);
        }
        
        PublishUpdatedInputs(input_sensor_type[param->port_input_pin_array_index], 
                             input_pin_number [param->port_input_pin_array_index], 
                             topic_value);

        param->pin_previous_states = CHECK_BIT(pin_current_states, param->port_pin_x);
    }
}

void ProcessUpdatedPins(struct input_parameters* param)
{
    String topic_value;
    u8 pin_current_states = *IOHW_GetPortPinStatus(param->port_name);
    
    if( ( (pin_current_states) ^ (param->pin_previous_states) ) != 0)
    {
        u8 current_pin_states = pin_current_states;
        u8 updated_pins       = current_pin_states ^ param->pin_previous_states;

        for (u8 pin_i = 0; pin_i <= 7; pin_i++)
        {
            if ( (((u8)(updated_pins >> (u8) pin_i)) & (u8)1) != (u8)0 )
            {
                if(param->sensor_type_e == CONTACT) // For Contact switches feedback like Doors, Windows & Motion
                {
                    ((current_pin_states >> pin_i) & 1) ? (topic_value = "OPEN") : (topic_value = "CLOSED");
                }
                else if(param->sensor_type_e == STATUS) // For AC status report feedback to OpenHAB. 
                {
                    ((current_pin_states >> pin_i) & 1) ? (topic_value = "OFF") : (topic_value = "ON"); // relay
                    pin_state_status[param->pin_input_start - 24 + pin_i] = ((current_pin_states >> pin_i) & 1); // put the current ac state in position pin_start - ( 24 which is first pin_start for ac state) + the pin_i
                }
                else
                {

                }

                PublishUpdatedInputs(input_sensor_type[pin_i + param->pin_input_start], 
                                     input_pin_number [pin_i + param->pin_input_start], 
                                     topic_value);
            }
        }
        param->pin_previous_states = current_pin_states;
    }
}

void PublishUpdatedInputs(String input_sensor_type_param, String input_pin_number_param, String topic_value)
{
    String topic_name  = "NA";

    topic_name = String(DEVICE_NAME) + "/" + input_sensor_type_param + "/" + input_pin_number_param + "/state";
    topic_name.toCharArray(mqtt_publish_topic_name, topic_name.length()+1);
    topic_value.toCharArray(mqtt_publish_topic_value, topic_value.length()+1);

    DEBUG_SERIAL(topic_name + " " );
    DEBUG_SERIAL_NL(topic_value);
#ifdef ETHERNET_CONN
    COMH_PublishMQTT(mqtt_publish_topic_name , mqtt_publish_topic_value);
#endif
}

u8* SENH_GetCurrentLedStates() 
{
    static u8 pin_states_param[18];
    
    for(int i = 0; i < 18; i++)
    {
        pin_states_param[i] = pin_state_status[i];
    }
 
    return pin_states_param;
}