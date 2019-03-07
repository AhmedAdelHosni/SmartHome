/******************************************************************************/
/*                Include common and project definition header                */
/******************************************************************************/

#include "Defined_Types.h"
#include "prj_pdf.h"

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/

#include "COMH.h"
#include "IoHw.h"
#include "SENH.h"
#include "LEDH.h"

/******************************************************************************/
/*                            Include other headers                           */
/******************************************************************************/

/******************************************************************************/
/*                   Definition of local symbolic constants                   */
/******************************************************************************/

/******************************************************************************/
/*                  Definition of local function like macros                  */
/******************************************************************************/

/******************************************************************************/
/*          Definition of local types (typedef, enum, struct, union)          */
/******************************************************************************/

/******************************************************************************/
/*                       Definition of local variables                        */
/******************************************************************************/

static unsigned long current_millis = millis();
static unsigned long previous_millis_50ms = 0;


/******************************************************************************/
/*                  Declaration of local function prototypes                  */
/******************************************************************************/

/******************************************************************************/
/*                        Declaration of local Objects                        */
/******************************************************************************/


/******************************************************************************/
/*                       Definition of local functions                        */
/******************************************************************************/

void setup()
{
    IOHW_ConfigurePorts();

  #if (IS_DEBUG_SERIAL != APPLICATION_DISABLED)
    IOHW_SerialBegin();
    IOHW_DisableSerialRX(); // override the Serial.begin and disable the Receiver since pin PE1 "USART0_RX" will be used.
  #endif

  #ifdef ETHERNET_CONN
    COMH_INIT();
    COMH_PublishMQTT("ETH0/State/", "System is started");
  #endif
    SENH_Init();
    LEDH_Init();
}

void loop(void)
{
    current_millis = millis();

    COMH_Cyclic();
    LEDH_Cyclic();

    if ((unsigned long)(current_millis - previous_millis_50ms) >= INTERVAL_PERIOD_50_MS)  // 50ms cyclic
    {
        previous_millis_50ms = millis();

        SENH_Cyclic50ms();
        LEDH_Cyclic50ms();
    }
}
