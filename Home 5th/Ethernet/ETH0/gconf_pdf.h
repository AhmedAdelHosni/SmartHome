#ifndef I_GCONF_PDF
#define I_GCONF_PDF

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* EDIT THE BELOW MACROS TO MATCH YOUR PROJECT CONFIGURATIONS AND SETTINGS */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*
 * Every device shall have a unique name. The DEVICE_NAME is a macro which is sent 
 * with the packet transmitted to the Home Automation Server to identify which 
 * Ethernet node is sending.
 */
#define DEVICE_NAME                     "E0"

/*
 * To Enable the Ethernet functionality in the SW a specific MACRO has to be defined.
 * #define ETHERNET_CONN                APPLICATION_ENABLED
 * To Disable the Ethernet functionality in the SW use the following
 * #define ETHERNET_CONN                APPLICATION_DISABLED 
 */
#define ETHERNET_CONN                   APPLICATION_ENABLED

/*
 * Define the MAC address for the Ethernet Sheild Module.
 * This shall be unique for each device.
 */
#define GCONF_MAC_ADDR                  {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

/*
 * Define the IP address for your Ethernet Sheild Module.
 * This shall be unique for each device.
 */
#define GCONF_IP_ADDR                   {192, 168, 1, 177};

/*
 * Define the IP address of you Home Automation Controller.
 * This Controller can be a Raspberry PI module running a MQTT broker.
 * The IP of this controller shall be static and fixed from the router settings.
 */
#define GCONF_CLIENT_ADDR               "192.168.1.3"

/*
 * Define the maximuim size of the MQTT subscribe topic.
 * The MQTT subscribe topic is defined in MQTT_TOPIC_SUBSCRIBE_NAME below. 
 */
#define MAX_MQTT_SUBSRIBE_SIZE          10

/*
 * Define the Broker name which will send the data from the controller to
 * the Ethernet module.
 */
#define MQTT_HA_BROKER_NAME             "OH"

/*
 * Define the MQTT topic subsribed to which will contain the data from the controller.
 */
#define MQTT_TOPIC_SUBSCRIBE_NAME       "/"MQTT_HA_BROKER_NAME"/"DEVICE_NAME"/#"

/*
 * To Enable Serial debugging in the SW a specific MACRO has to be defined.
 * #define IS_DEBUG_SERIAL APPLICATION_ENABLED
 * To Disable the Serial debugging in the SW use the following
 * #define IS_DEBUG_SERIAL APPLICATION_DISABLED
 */
#define IS_DEBUG_SERIAL                 APPLICATION_ENABLED

/* 
 * Define Serial Transmission Baud Rate 
 */
#define GCONF_BAUD_RATE                  115200

/* 
 * During development it may be required to test the LEDH output on normal diode LED.
 * Definig the MACRO as APPLICATION_ENABLED will set the output pin as High or Low 
 * if the command received from the Home automation system is 1 or 0.
 * The LEDH will not check for the current pin status.
 * Defining the MACRO as APPLICATION_DISABLED is the correct value during production.
 * The LEDH will check the current pin status and toggle the pin output accordingly.
 */
#define APPL_ENABLE_LEDH_TEST           APPLICATION_ENABLED

/* 
 * This defines the delay between executing mutiple LED commands received at one common 
 * MQTT request cycle.
 */
#define LED_STATE_UPDATE_INTERVAL       200u

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* END OF PROJECT CONFIGURATIONS AND SETTINGS. DO NOT CHANGE BELOW LINES */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/*
 * This defines the inteval period counter for a 50 ms cyclic task.
 * This MACRO is used in the main loop function. 
 */
#define     INTERVAL_PERIOD_50_MS       50

/*
 * To Enable Serial debugging in the SW a specific MACRO has to be defined.
 * #define IS_DEBUG_SERIAL APPLICATION_ENABLE
 * To Disable the Serial debugging in the SW use the following
 * #define IS_DEBUG_SERIAL APPLICATION_DISABLED
 */
#if (IS_DEBUG_SERIAL != APPLICATION_DISABLED)   
    #define DEBUG_SERIAL(x)            Serial.print(x)   
#else
    #define DEBUG_SERIAL(x)                           
#endif                                          

#if (IS_DEBUG_SERIAL != APPLICATION_DISABLED)   
    #define DEBUG_SERIAL_NL(x)            Serial.println(x)   
#else
    #define DEBUG_SERIAL_NL(x)                           
#endif   

/* DELAY_MS */
#define DELAY_MS                            delay



#endif /* I_GCONF_PDF */
