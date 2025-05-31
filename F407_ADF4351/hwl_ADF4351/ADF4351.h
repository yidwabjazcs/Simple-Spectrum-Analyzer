#ifndef _ADF4351_H_
#define _ADF4351_H_
#include "stm32f4xx_hal.h"

//#define ADF4351_CE PEout(2)
#define ADF4351_CE_Set() HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_SET)
#define ADF4351_CE_Clr() HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_RESET)

//#define ADF4351_LE PEout(3)
#define ADF4351_LE_Set() HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET)
#define ADF4351_LE_Clr() HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET)

//#define ADF4351_OUTPUT_DATA PEout(4)
#define ADF4351_OUTPUT_DATA_Set() HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET)
#define ADF4351_OUTPUT_DATA_Clr() HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET)

//#define ADF4351_CLK		PEout(5)
#define ADF4351_CLK_Set() HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET)
#define ADF4351_CLK_Clr() HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET)

#define ADF4351_RF_OFF	((uint32_t)0XEC801C)
#define StrMax    10//扫频缓存数据大小

void ADF4351Init(void);
void WriteToADF4351(uint8_t count, uint8_t *buf);
void WriteOneRegToADF4351(uint32_t Regster);
void ADF4351_Init_some(void);
void delay_us(uint32_t delay_us);

void ADF4351WriteFreq(float Fre);		//	(xx.x) MHz 点频

#endif

