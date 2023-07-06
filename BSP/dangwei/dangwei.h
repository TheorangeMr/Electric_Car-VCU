#ifndef _DANGWEI_H
#define _DANGWEI_H

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

#define KA8								GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)
#define KE5								GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5)
#define KE6								GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6)

uint8_t Key_scan(void);
void dangwei_gpio_init(void);

#endif

