 #include "adc.h"
 
/* 
	*��������Adc1_Init()
	*��  �ܣ�ADC��ʼ������
	*��  ������
	*����ֵ����
*/
void Adc1_Init(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1, ENABLE );	  //ʹ��ADC1ͨ��ʱ��

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6  72M/6=12,ADC���ʱ�䲻�ܳ���14M

	/*����ADCͨ������Ϊģ������*/                       
	GPIO_InitStructure.GPIO_Pin = ADC_CH1_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(ADC_CH1_GPIO_PORT, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  //��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ

  /*��ʼ��ADCת��ģʽ*/
  #if ADC1_SCAN_ENABLE
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	                    //ADC�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;	                          //����ɨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                      //��������ת��
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	    //ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	                //ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = ADC1_SCAN_CHANNEL_NUM;	            //ɨ���ͨ����
	ADC_Init(ADC1, &ADC_InitStructure);	                                    //����ADC_InitStruct��ָ���Ĳ�����ʼ������ADC1�ļĴ���   
  #else
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                      //ADC�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;                           //����ɨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                     //����������ת��
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;     //ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                  //ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;                                 //˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);                                     //����ADC_InitStruct��ָ���Ĳ�����ʼ������ADC1�ļĴ���   
	#endif
  
  /*���ù������ͨ������*/
	#if ADC1_SCAN_ENABLE
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7 , ADC_SampleTime_239Cycles5);
	#endif
  
	ADC_Cmd(ADC1, ENABLE);	//ʹ��ָ����ADC1
	
	ADC_ResetCalibration(ADC1);	//ʹ�ܸ�λУ׼  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//�ȴ���λУ׼����
	
	ADC_StartCalibration(ADC1);	 //����ADУ׼
 
	while(ADC_GetCalibrationStatus(ADC1));	 //�ȴ�У׼����
 
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������
}				  
/* 
	*��������Adc2_Init()
	*��  �ܣ�ADC��ʼ������
	*��  ������
	*����ֵ����
*/
void Adc2_Init(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC2, ENABLE );	  //ʹ��ADC2ͨ��ʱ��

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6  72M/6=12,ADC���ʱ�䲻�ܳ���14M

	/*����ADCͨ������Ϊģ������*/                       
	GPIO_InitStructure.GPIO_Pin = ADC_CH2_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(ADC_CH2_GPIO_PORT, &GPIO_InitStructure);	

	ADC_DeInit(ADC2);  //��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ

  /*��ʼ��ADCת��ģʽ*/
  #if ADC2_SCAN_ENABLE
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	                    //ADC�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;	                          //����ɨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                      //��������ת��
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	    //ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	                //ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = ADC2_SCAN_CHANNEL_NUM;	            //ɨ���ͨ����
	ADC_Init(ADC2, &ADC_InitStructure);	                                    //����ADC_InitStruct��ָ���Ĳ�����ʼ������ADC1�ļĴ���   
  #else
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                      //ADC�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;                           //����ɨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                     //����������ת��
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;     //ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                  //ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;                                 //˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC2, &ADC_InitStructure);                                     //����ADC_InitStruct��ָ���Ĳ�����ʼ������ADC1�ļĴ���   
	#endif
  
  /*���ù������ͨ������*/
	#if ADC2_SCAN_ENABLE
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 1 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 2 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_2, 3 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 4 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_4, 5 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 6 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 7 , ADC_SampleTime_239Cycles5);
	#endif
  
	ADC_Cmd(ADC2, ENABLE);	//ʹ��ָ����ADC1
	
	ADC_ResetCalibration(ADC2);	//ʹ�ܸ�λУ׼  
	 
	while(ADC_GetResetCalibrationStatus(ADC2));	//�ȴ���λУ׼����
	
	ADC_StartCalibration(ADC2);	 //����ADУ׼
 
	while(ADC_GetCalibrationStatus(ADC2));	 //�ȴ�У׼����
 
//	ADC_SoftwareStartConvCmd(ADC2, ENABLE);		//ʹ��ָ����ADC2�����ת����������
}				  
/* 
	*��������ADC_SingleMode_GetAverageValue()
	*��  �ܣ���ȡ����ģʽADCֵ
	*��  ������
	*����ֵ��Average
*/
uint16_t ADC_SingleMode_GetAverageValue(ADC_TypeDef* ADCx,u8 channelx)
{
  u8 t;
  u16 temp = 0;
  u16 Average = 0;
  u32 Sum_Value = 0;
  u16 Max_Value = 0;
	u16 Min_Value = 4095;
  //����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADCx, channelx, 1, ADC_SampleTime_239Cycles5);	//ADC1,ADCͨ��,����ʱ��Ϊ239.5����	  			    

  for(t=0;t<Filter_times;t++)
	{
    ADC_SoftwareStartConvCmd(ADCx, ENABLE);		      //ʹ��ָ����ADC1�����ת����������	
    
    while(!ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC));  //�ȴ�ת������
    
    temp = ADC_GetConversionValue(ADCx);            //��ȡ���һ��ADC1�������ת�����
    
    Max_Value = (Max_Value>temp)?Max_Value:temp;
		Min_Value = (Min_Value<temp)?Min_Value:temp;
		Sum_Value += temp;
	}
  /*ȥ�����ֵ����Сֵ����ƽ��ֵ*/
	Average = (Sum_Value - Max_Value - Min_Value)/(Filter_times-2);
	return Average;	
}

/* 
	*��������ADC_ScanMode_GetAverageVlaue()
	*��  �ܣ���ȡɨ��ģʽADCֵ
	*��  ����*zdata������ָ��
	*����ֵ����
*/
/* 
	���ʹ����ɨ��ģʽ��DMA������һ��ADC_Value[][]�������洢ADת�������
	��ADC1_SCAN_CHANNEL_NUM��ͨ����ÿ��ͨ������Filter_times�����ݡ�
*/
#if ADC1_SCAN_ENABLE && ADC1_DMA_ENABLE
vu16 ADC_Value[Filter_times][ADC1_SCAN_CHANNEL_NUM] = {0};
#endif

#if ADC1_SCAN_ENABLE

void ADC_ScanMode_GetAverageVlaue(u16 *zdata)
{
	u32 Sum = 0;
	u16 Max = 0;
	u16 Min = 4095;
  u8 i = 0,j = 0;
  
	/* �������ת�� */
	ADC_SoftwareStartConvCmd(ADC1 , ENABLE);
	
	/* �ȴ�ת����� */
	//while(ADC_GetFlagStatus(ADC1 , ADC_FLAG_EOC) == RESET);
	//���ڵȴ�ת����ɣ���ʹ��ʵʱ����ϵͳʱ������ʹ����ʱ��������CPUʹ��Ȩ��
	//�Ӷ������γ�����
	delay_ms(10);

	/* ����˲�����ֵ�˲��� */ 
	for(i = 0; i < ADC1_SCAN_CHANNEL_NUM; i++)
	{
		Sum = 0;
		Max = 0;
		Min = 4095;
		for(j = 0; j < Filter_times; j++)
		{
			Max = (Max>ADC_Value[j][i])?Max:ADC_Value[j][i];
		  Min = (Min<ADC_Value[j][i])?Min:ADC_Value[j][i];
			Sum += ADC_Value[j][i];
		}
		zdata[i] = (Sum-Min-Max)/(Filter_times-2);
	}
}

#endif


























