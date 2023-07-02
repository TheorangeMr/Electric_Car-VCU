#ifndef __ADC_H
#define __ADC_H	

#include "stm32f10x.h"

#define ADC1_SCAN_ENABLE       0         //ADC1ɨ��ģʽʹ��
#define ADC1_DMA_ENABLE        0         //ADC1��DMAʹ��
#define ADC1_SCAN_CHANNEL_NUM  7         //ɨ���ͨ����
#define ADC2_SCAN_ENABLE       0         //ADC2ɨ��ģʽʹ��
#define ADC2_DMA_ENABLE        0         //ADC2��DMAʹ��
#define ADC2_SCAN_CHANNEL_NUM  7         //ɨ���ͨ����
#define Filter_times           5         //ADC�˲�����

#define PCM300D_Constant_Pa        25.0*10e6/3300
#define PCM300D_Constant_MPa       25.0/3.3
/************************************************************
								ADC�ⲿͨ��������ź궨��
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
								        ADC���ܺ���
************************************************************/
void Adc1_Init(void);
void Adc2_Init(void);
uint16_t ADC_SingleMode_GetAverageValue(ADC_TypeDef* ADCx,u8 channelx);
void ADC_ScanMode_GetAverageVlaue(u16 *zdata);

#endif 
