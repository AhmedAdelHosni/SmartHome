#ifndef I_COMH
#define I_COMH

void COMH_INIT(void);
void COMH_PublishMQTT(const String topic, const String payload);
u32 COMH_GetRequestedLedStates(void);
u32 COMH_GetConfirmedLedStates(void);


#endif /* I_COMH */