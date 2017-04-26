#include "bsp.h"
#include "lis3dx.h"
#include "s2l.h"
#include "string.h"

extern TIM_HandleTypeDef htim6;
static void LteInit();

void BspInit()
{
  memset(&car,0,sizeof(car));
  ActiveDevice();
  CheckActive();
  //close the all isr 
  MwCtrl(1);
  //
  //__disable_irq();
  HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);
  
  Lis3dxInit();
  LteInit();
  //TimeStart();
  //on
  //HAL_GPIO_WritePin(ON_OFF_PORT,ON_OFF,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IR_EN2_PORT,IR_EN2,GPIO_PIN_SET);
  car.engine=1;
  car.gps=1;
  car.mw=1;
  car.plan=1;
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}

static void (*TimeCallback)();
void TimeStart(void (*func)(),uint16_t time)
{
  TimeSetTimeout(time);
  HAL_TIM_Base_Start_IT(&htim6);
  TimeCallback=func;
}
void TimeStop()
{
  HAL_TIM_Base_Stop_IT(&htim6);
  TimeCallback=NULL;
}
//ms
void TimeSetTimeout(uint16_t period)
{
  htim6.Instance->ARR=period;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(TimeCallback)
    TimeCallback();
}


void MwCtrl(uint8_t on)
{
  if(on)
    HAL_GPIO_WritePin(MV_EN_PORT,MV_EN,GPIO_PIN_RESET);
  else
    HAL_GPIO_WritePin(MV_EN_PORT,MV_EN,GPIO_PIN_SET);
}
void LedTog(uint8_t num)
{
  switch(num)
  {
  case 0:
    {
      HAL_GPIO_TogglePin(LED1_PORT,LED1);
      break;
    }
  case 1:
    {
      HAL_GPIO_TogglePin(LED2_PORT,LED2);
      break;
    }
  case 2:
    {
      HAL_GPIO_TogglePin(LED3_PORT,LED3);
      break;
    }
  default:
    break;
    
  }
}
void LedSet(uint8_t num,uint8_t state)
{
  switch(num)
  {
  case 0:
    {
      HAL_GPIO_WritePin(LED1_PORT,LED1,(GPIO_PinState)state);
      break;
    }
  case 1:
    {
      HAL_GPIO_WritePin(LED2_PORT,LED2,(GPIO_PinState)state);
      break;
    }
  case 2:
    {
      HAL_GPIO_WritePin(LED3_PORT,LED3,(GPIO_PinState)state);
      break;
    }
  default:
    break;
    
  }
}
/***************
x:0,1,2,3
0:2.13
1:1.96
2:1.96
3:1.88

***************/
void PirLevelSet(uint8_t x)
{
  HAL_GPIO_WritePin(PIR_S1_PORT,PIR_S1,(GPIO_PinState)(x&0x01));
  HAL_GPIO_WritePin(PIR_S2_PORT,PIR_S2,(GPIO_PinState)(x&0x02));
}
//void LteStop()
//{
//  HAL_GPIO_WritePin(LTE_CLOSE_PORT,LTE_CLOSE,GPIO_PIN_SET);//OPEN
//}
static void LteInit()
{
  HAL_GPIO_WritePin(LTE_CLOSE_PORT,LTE_CLOSE,GPIO_PIN_RESET);//close
  HAL_GPIO_WritePin(LTE_WAKEUP_PORT,LTE_WAKEUP_PIN,GPIO_PIN_RESET);//AUTO SLEEP
  HAL_GPIO_WritePin(LTE_RESET_PORT,LTE_RESET,GPIO_PIN_SET);//NO RESET
}
void LteReset()
{
  HAL_GPIO_WritePin(LTE_RESET_PORT,LTE_RESET,GPIO_PIN_RESET);//NO RESET
  osDelay(2000);
  HAL_GPIO_WritePin(LTE_RESET_PORT,LTE_RESET,GPIO_PIN_SET);//NO RESET
}
void LteRestart()
{
  HAL_GPIO_WritePin(LTE_CLOSE_PORT,LTE_CLOSE,GPIO_PIN_SET);//close
  osDelay(1000);
  HAL_GPIO_WritePin(LTE_CLOSE_PORT,LTE_CLOSE,GPIO_PIN_RESET);//close
}
void PowerS2l(uint8_t in)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(in==0)
  {
    
    GPIO_InitStruct.Pin = ON_OFF;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ON_OFF_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(ON_OFF_PORT,ON_OFF,GPIO_PIN_RESET);//close
  }
  else
  {
    GPIO_InitStruct.Pin = ON_OFF;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ON_OFF_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(ON_OFF_PORT,ON_OFF,GPIO_PIN_SET);//close
    osDelay(200);
    GPIO_InitStruct.Pin = ON_OFF;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ON_OFF_PORT, &GPIO_InitStruct);
  }
}
void LteOpen()
{
  HAL_GPIO_WritePin(LTE_CLOSE_PORT,LTE_CLOSE,GPIO_PIN_RESET);
}

void ActiveDevice()
{
  if(!car.active)
  {
    HAL_FLASHEx_DATAEEPROM_Unlock();
    car.active=1;
    HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, NVM_ACTIVE,1);
    HAL_FLASHEx_DATAEEPROM_Lock();
  }
}
void DeActiveDevice()
{
  if(car.active)
  {
    HAL_FLASHEx_DATAEEPROM_Unlock();
    car.active=0;
    HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, NVM_ACTIVE,0);
    HAL_FLASHEx_DATAEEPROM_Lock();
    HAL_NVIC_SystemReset();
  }
}
void CheckActive()
{
  uint32_t data;
  data=*(__IO uint32_t *)NVM_ACTIVE;
  if(data)
    car.active=1;
  else
    car.active=0;
}
extern UART_HandleTypeDef huart1;
int fputc(int ch, FILE *f)
{

 

  uint8_t a;

  a=(uint8_t)ch;

  HAL_UART_Transmit(&huart1, &a, 1, 100);

  return ch;

}