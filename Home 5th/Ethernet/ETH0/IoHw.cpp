/******************************************************************************/
/*                Include common and project definition header                */
/******************************************************************************/
#include "Defined_Types.h"
#include "prj_pdf.h"

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/

#include "IoHw.h"

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

/******************************************************************************/
/*                  Declaration of local function prototypes                  */
/******************************************************************************/

/******************************************************************************/
/*                        Declaration of local Objects                        */
/******************************************************************************/

/******************************************************************************/
/*                       Definition of local functions                        */
/******************************************************************************/

void IOHW_ConfigurePorts(void)
{
    PORTA = 0xFF;       // Set port A pin 7 6 5 4 3 2 1 0 as pull up for MEGA
    PORTC = 0xFF;       // Set port C pin 7 6 5 4 3 2 1 0 as pull up for MEGA
    PORTL = 0xFF;       // Set port L pin 7 6 5 4 3 2 1 0 as pull up for MEGA
    PORTF = 0xFF;       // Set port A pin 7 6 5 4 3 2 1 0 as pull up for MEGA (( ANALOUGE)
    PORTK = 0xFF;       // Set port A pin 7 6 5 4 3 2 1 0 as pull up for MEGA (( ANALOUGE)
    PORTD |= B10000000; // Set port D pin 7               as pull up for MEGA
    PORTG |= B00000111; // Set port G pin           2 1 0 as pull up for MEGA
    
    DDRE |= B00111001;  // Set port E bit 0 3 4 5 to Output
    DDRG |= B00100000;  // Set port G bit 4 to Output
    DDRH |= B01111011;  // Set port H bit 0 1 to Output
    DDRB |= B11100000;  // Set port B bit 5 6 7 to Output
    DDRJ |= B00000011;  // Set port J bit 0 1 to Output
    DDRD |= B00001100;  // Set port D bit 2 3 to Output
}
#if (IS_DEBUG_SERIAL != APPLICATION_DISABLED)   
void IOHW_SerialBegin(void)
{
  Serial.begin(GCONF_BAUD_RATE);
}
#endif

#if (IS_DEBUG_SERIAL != APPLICATION_DISABLED)
void IOHW_DisableSerialRX(void)
{
  cbi(UCSR0B, RXEN0);
  cbi(UCSR0B, RXCIE0);
}
#endif

void IOHW_OutputHigh(u8 * port_name, u8 pin)
{  
  *port_name |= (u8)((u8)1<<(u8) pin);
}

void IOHW_OutputLow(u8 * port_name, u8 pin)
{
  *port_name &= ~(1 <<(u8) pin);
}

void IOHW_TogglePin(uint16_t* port_name, u8 pin)
{
  *port_name ^= (1UL << pin);
}

u16 * IOHW_GetPortPinStatus(enum IOHW_port_pins_x port_name)
{
  u16 port_status = 0;

  switch (port_name)
  {
    case PORT_A:
      port_status = port_to_input_PGM[PORT_A];
      break;
    case PORT_C:
      port_status = port_to_input_PGM[PORT_C];
      break;
    case PORT_L:
      port_status = port_to_input_PGM[PORT_L];
      break;
    case PORT_F:
      port_status = port_to_input_PGM[PORT_F];
      break;
    case PORT_K:
      port_status = port_to_input_PGM[PORT_K];
      break;
    case PORT_D:
      port_status = port_to_input_PGM[PORT_D];
      break;
    case PORT_G:
      port_status = port_to_input_PGM[PORT_G];
      break;

    default:
      port_status = 0;
      break;
  }
  return port_status;
}
