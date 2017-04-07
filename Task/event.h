#ifndef _EVENT_H
#define _EVENT_H


#include "bsp.h"
#include "stm32l0xx_hal.h"

#define UPLOAD_EVENT_ACCEL 0X0001
#define UPLOAD_EVENT_MW    0X0002
#define UPLOAD_EVENT_LTE   0X0004
#define UPLOAD_EVENT_TOKEN 0x0008


typedef enum
{
  EvtPower=0U,
  EvtAccel1,
  EvtAceel2,
  EvtMicroWave,
  EvtLte,
  EvtPir1,
  EvtPir2
}EventEnum;

typedef struct
{
  EventEnum evt;
  uint32_t tick;
  GPIO_PinState io;
}EventTypeDef;

void EventTask();
void SetEvent(uint8_t event,uint8_t on_off);
#endif