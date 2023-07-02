#ifndef __ADC_H
#define __ADC_H	

#include "stm32f10x.h"

#define ADC1_SCAN_ENABLE       0         //ADC1扫描模式使能
#define ADC1_DMA_ENABLE        0         //ADC1的DMA使能
#define ADC1_SCAN_CHANNEL_NUM  7         //扫描的通道数
#define ADC2_SCAN_ENABLE       0         //ADC2扫描模式使能
#define ADC2_DMA_ENABLE        0         //ADC2的DMA使能
#define ADC2_SCAN_CHANNEL_NUM  7         //扫描的通道数
#define Filter_times           5         //ADC滤波次数

#define PCM300D_Constant_Pa        25.0*10e6/3300
#define PCM300D_Constant_MPa       25.0/3.3
/************************************************************
								ADC外部通道相关引脚宏定义
************************************************************/
/* ADC channel_0 */
#define  ADC_CH0_GPIO_PORT       GPIOA   
#define  ADC_CH0_GPIO_PIN        GPIO_Pin_0
/* ADC channel_1 */
#define  ADC_CH1_GPIO_PORT       GPIOA   
#define  ADC_CH1_GPIO_PIN        GPIO_Pin_1
/* ADC channel_2 */
#define  ADC_CH2_GPIO_PORT       GPIOA   
#define  ADC_CH2_GPIO_PIN        GPIO_Pin_2
/* ADC channel_3 */
#define  ADC_CH3_GPIO_PORT       GPIOA   
#define  ADC_CH3_GPIO_PIN        GPIO_Pin_3
/* ADC channel_4 */
#define  ADC_CH4_GPIO_PORT       GPIOA   
#define  ADC_CH4_GPIO_PIN        GPIO_Pin_4
/* ADC channel_5 */
#define  ADC_CH5_GPIO_PORT       GPIOA   
#define  ADC_CH5_GPIO_PIN        GPIO_Pin_5
/* ADC channel_6 */
#define  ADC_CH6_GPIO_PORT       GPIOA   
#define  ADC_CH6_GPIO_PIN        GPIO_Pin_6
/* ADC channel_7 */
#define  ADC_CH7_GPIO_PORT       GPIOA   
#define  ADC_CH7_GPIO_PIN        GPIO_Pin_7
/* ADC channel_8 */
#define  ADC_CH8_GPIO_PORT       GPIOB   
#define  ADC_CH8_GPIO_PIN        GPIO_Pin_0
/* ADC channel_9 */
#define  ADC_CH9_GPIO_PORT       GPIOB   
#define  ADC_CH9_GPIO_PIN        GPIO_Pin_1
/* ADC channel_10 */
#define  ADC_CH10_GPIO_PORT      GPIOC   
#define  ADC_CH10_GPIO_PIN       GPIO_Pin_0
/* ADC channel_11 */
#define  ADC_CH11_GPIO_PORT      GPIOC   
#define  ADC_CH11_GPIO_PIN       GPIO_Pin_1
/* ADC channel_12 */
#define  ADC_CH12_GPIO_PORT      GPIOC   
#define  ADC_CH12_GPIO_PIN       GPIO_Pin_2
/* ADC channel_13 */
#define  ADC_CH13_GPIO_PORT      GPIOC   
#define  ADC_CH13_GPIO_PIN       GPIO_Pin_3
/* ADC channel_14 */
#define  ADC_CH14_GPIO_PORT      GPIOC   
#define  ADC_CH14_GPIO_PIN       GPIO_Pin_4
/* ADC channel_15 */
#define  ADC_CH15_GPIO_PORT      GPIOC   
#define  ADC_CH15_GPIO_PIN       GPIO_Pin_5

/************************************************************
								        ADC功能函数
************************************************************/
void Adc1_Init(void);
void Adc2_Init(void);
uint16_t ADC_SingleMode_GetAverageValue(ADC_TypeDef* ADCx,u8 channelx);
void ADC_ScanMode_GetAverageVlaue(u16 *zdata);

#endif 
