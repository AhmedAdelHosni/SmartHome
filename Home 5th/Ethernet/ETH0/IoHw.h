#ifndef I_IOHW
#define I_IOHW

enum IOHW_port_pins_x {
  UNUSED_PORT_X_0,
  PORT_A,
  PORT_B,
  PORT_C,
  PORT_D,
  PORT_E,
  PORT_F,
  PORT_G,
  PORT_H,
  UNUSED_PORT_X_1,
  PORT_J,
  PORT_K,
  PORT_L
};

void IOHW_ConfigurePorts(void);
void IOHW_DisableSerialRX(void);
void IOHW_SerialBegin(void);
void IOHW_TogglePin(uint16_t* port_name, u8 pin);
void IOHW_OutputLow(u8 * port_name, u8 pin);
void IOHW_OutputHigh(u8 * port_name, u8 pin);
u16 * IOHW_GetPortPinStatus(enum IOHW_port_pins_x port_name);

#endif /* I_IOHW */
