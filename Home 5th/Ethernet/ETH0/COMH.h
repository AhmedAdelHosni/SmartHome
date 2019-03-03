#ifndef I_COMH
#define I_COMH

void COMH_INIT(void);
void COMH_Cyclic(void);
void COMH_PublishMQTT(const String topic, const String payload);
void COMH_ClearRequestedLed(u8 relay_index);
u32 COMH_GetRequestedLedStateValues(void);
u32 COMH_GetRequestedLeds(void);


#endif /* I_COMH */