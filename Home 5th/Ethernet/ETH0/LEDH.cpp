/******************************************************************************/
/*                Include common and project definition header                */
/******************************************************************************/
#include "Defined_Types.h"
#include "prj_pdf.h"

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/

#include "LEDH.h"

/******************************************************************************/
/*                            Include other headers                           */
/******************************************************************************/

#include "SENH.h"
#include "COMH.h"
#include "IoHw.h"

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

static u8 port_pin[]       = { 0, 3, 4, 5,
                               5,
                               0, 1, 3, 4, 5, 6,
                               5, 6, 7,
                               0, 1,
                               2, 3};

const uint16_t port_name[] = { &PORTE, &PORTE, &PORTE, &PORTE, 
                               &PORTG, 
                               &PORTH, &PORTH, &PORTH, &PORTH, &PORTH, &PORTH,
                               &PORTB, &PORTB, &PORTB,
                               &PORTJ, &PORTJ,
                               &PORTD, &PORTD};

static u32 previous_led_states = 0;
static u32 current_led_states  = 0;

static u8 * ac_states = 0;

/******************************************************************************/
/*                  Declaration of local function prototypes                  */
/******************************************************************************/


void TurnOnRelay(u8 pin_n);
void TurnOffRelay(u8 pin_n);

/******************************************************************************/
/*                        Declaration of local Objects                        */
/******************************************************************************/

/******************************************************************************/
/*                       Definition of local functions                        */
/******************************************************************************/



void LEDH_Input(void)
{
    ac_states  = SENH_GetLedStates();
}

void LEDH_Cyclic(void)
{
    current_led_states  = COMH_GetLedStates();

    if(current_led_states != previous_led_states)
    {    
        u32 updated_pins = current_led_states ^ previous_led_states;

        for (u8 pin_i = 0; pin_i <= 17; pin_i++)
        {
            if ( (((u32)(updated_pins >> (u32) pin_i)) & (u32)1) != (u32)0 )
            {
                if(((current_led_states >> pin_i) & 1) != FALSE)
                {
                    TurnOnRelay(pin_i);
                }
                else
                {
                    TurnOffRelay(pin_i);
                }                
            }
        }
        previous_led_states = current_led_states;
    }
}

void TurnOnRelay(u8 pin_n)
{
    if(ac_states[pin_n] == FALSE)
    {
        IOHW_TogglePin(port_name[pin_n], port_pin[pin_n]);
    }
}

void TurnOffRelay(u8 pin_n)
{
    if(ac_states[pin_n] == TRUE)
    {
        IOHW_TogglePin(port_name[pin_n], port_pin[pin_n]);
    }
}