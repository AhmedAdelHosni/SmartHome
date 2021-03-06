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

#define CYCLE_TIME                          50u
#define MAX_NUM_OF_OUTPUTS                  18u
#define DEBOUNCE_LED_STATE_INTERVAL         LED_STATE_UPDATE_INTERVAL / CYCLE_TIME

#define LEDON                               0u
#define LEDOFF                              1u

#define REQUESTED_LEDS_NONE                 0u
/******************************************************************************/
/*          Definition of local types (typedef, enum, struct, union)          */
/******************************************************************************/

/******************************************************************************/
/*                       Definition of local variables                        */
/******************************************************************************/

// TODO : implement in LEDH_Init() all initializations

static u8 new_led_states [18] = {0};

static u8 port_pin[]         = { 0, 3, 4, 5,
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

static u32 previous_led_states   = 0;
static u32 requested_led_state_values  = 0;
static u32 requested_leds  = 0;

static u8 * ac_states;

static bool_T is_new_led_states_available = FALSE;

static u8 led_state_index_counter = 0;
static u8 led_state_debounce = 0;

/******************************************************************************/
/*                  Declaration of local function prototypes                  */
/******************************************************************************/


void LEDH_Input(void);
void LEDH_Process(void);
void LEDH_Output(void);
void TurnOnRelay(u8 pin_n);
void TurnOffRelay(u8 pin_n);
void UpdateNewLedStates(u8 pin_i, u8 new_state);

/******************************************************************************/
/*                        Declaration of local Objects                        */
/******************************************************************************/

/******************************************************************************/
/*                       Definition of local functions                        */
/******************************************************************************/

void LEDH_Init(void)
{

}

void LEDH_Cyclic(void)
{
    if(requested_leds != REQUESTED_LEDS_NONE)
    {
        if(led_state_index_counter < MAX_NUM_OF_OUTPUTS)
        {
            if ( (((u32)(requested_leds >> (u32) led_state_index_counter)) & (u32)1) == (u32)0 )
            {
                led_state_index_counter++;
            }
        }
        else
        {
            led_state_index_counter = 0;
        }
    }
}

void LEDH_Cyclic50ms(void)
{
    LEDH_Input();
    LEDH_Process();
    LEDH_Output();
}

void LEDH_Input(void)
{
    ac_states  = SENH_GetCurrentLedStates();     
    requested_leds  = COMH_GetRequestedLeds();
    requested_led_state_values  = COMH_GetRequestedLedStateValues();
}

void LEDH_Process(void)
{

}

void LEDH_Output(void)
{
    if(requested_leds == REQUESTED_LEDS_NONE)
    {
        led_state_index_counter = 0;
    }
    else
    {
        if(led_state_index_counter < MAX_NUM_OF_OUTPUTS)
        {
            if(led_state_debounce < DEBOUNCE_LED_STATE_INTERVAL)
            {
                led_state_debounce = led_state_debounce + 1;
            }
            else
            {
                if ( (((u32)((u32)requested_leds >> (u32) led_state_index_counter)) & (u32)1) != (u32)0 )
                {
                    if ( (((u32)((u32)requested_led_state_values >> (u32) led_state_index_counter)) & (u32)1) != (u32)0 )
                    {
                        TurnOnRelay(led_state_index_counter);
                    }
                    else
                    {
                        TurnOffRelay(led_state_index_counter);
                    }
                    // disable interrupts here.
                    COMH_ClearRequestedLed(led_state_index_counter);
                    requested_leds &= ~((u32)1 <<(u32) led_state_index_counter);
                    led_state_index_counter = led_state_index_counter + 1;
                }    
                
                led_state_debounce = 0;    
            }
        }
    }
}

void UpdateNewLedStates(u8 pin_i, u8 new_state)
{
    new_led_states[pin_i] = new_state;
}

void TurnOnRelay(u8 pin_n)
{
#if APPL_ENABLE_LEDH_TEST != APPLICATION_ENABLED
    if(ac_states[pin_n] == LEDON)
    {
        IOHW_TogglePin(port_name[pin_n], port_pin[pin_n]);
    }
#else
        IOHW_OutputHigh(port_name[pin_n], port_pin[pin_n]);
#endif
}

void TurnOffRelay(u8 pin_n)
{
#if APPL_ENABLE_LEDH_TEST != APPLICATION_ENABLED
    if(ac_states[pin_n] == LEDOFF)
    {
        IOHW_TogglePin(port_name[pin_n], port_pin[pin_n]);
    }
#else
        IOHW_OutputLow(port_name[pin_n], port_pin[pin_n]);
#endif
}
