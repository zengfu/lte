#ifndef S2L_h
#define S2L_H

#include "stm32l0xx_hal.h"

typedef struct
{
  //rw
  uint8_t gps:1;//LSB
  uint8_t mw:1;
  uint8_t plan:1;
  //read only
  uint8_t engine:1; 
  uint8_t card:1;
  uint8_t token:1;
}CarTypeDef;

void S2lTask();
uint16_t crc16(uint8_t data,uint16_t crc);

extern CarTypeDef car;




#endif