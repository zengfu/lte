#ifndef S2L_h
#define S2L_H

#include "stm32l0xx_hal.h"

typedef struct
{
  int engine:1;  //0 off 1 on
  int gps:1;
  int mw:1;
  int card:1;
  int accel:1;
  int token:1;
  int plan:1;
}CarTypeDef;

void S2lTask();
uint16_t crc16(uint8_t data,uint16_t crc);





#endif