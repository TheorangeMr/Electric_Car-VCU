 #include "adc.h"
 
/* 
	*函数名：Adc1_Init()
	*功  能：ADC初始化函数
	*参  数：无
	*返回值：无
*/
void Adc1_Init(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1, ENABLE );	  //使能ADC1通道时钟

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6  72M/6=12,ADC最大时间不能超过14M

	/*设置ADC通道引脚为模拟输入*/                       
	GPIO_InitStructure.GPIO_Pin = ADC_CH1_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(ADC_CH1_GPIO_PORT, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值

  /*初始化ADC转换模式*/
  #if ADC1_SCAN_ENABLE
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	                    //ADC工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;	                          //启动扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                      //启动连续转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	    //转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	                //ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = ADC1_SCAN_CHANNEL_NUM;	            //扫描的通道数
	ADC_Init(ADC1, &ADC_InitStructure);	                                    //根据ADC_InitStruct中指定的参数初始化外设ADC1的寄存器   
  #else
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                      //ADC工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;                           //启动扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                     //不启动连续转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;     //转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                  //ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;                                 //顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);                                     //根据ADC_InitStruct中指定的参数初始化外设ADC1的寄存器   
	#endif
  
  /*配置规则组各通道参数*/
	#if ADC1_SCAN_ENABLE
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7 , ADC_SampleTime_239Cycles5);
	#endif
  
	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	
	ADC_ResetCalibration(ADC1);	//使能复位校准  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束
	
	ADC_StartCalibration(ADC1);	 //开启AD校准
 
	while(ADC_GetCalibrationStatus(ADC1));	 //等待校准结束
 
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能
}				  
/* 
	*函数名：Adc2_Init()
	*功  能：ADC初始化函数
	*参  数：无
	*返回值：无
*/
void Adc2_Init(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC2, ENABLE );	  //使能ADC2通道时钟

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6  72M/6=12,ADC最大时间不能超过14M

	/*设置ADC通道引脚为模拟输入*/                       
	GPIO_InitStructure.GPIO_Pin = ADC_CH2_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(ADC_CH2_GPIO_PORT, &GPIO_InitStructure);	

	ADC_DeInit(ADC2);  //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值

  /*初始化ADC转换模式*/
  #if ADC2_SCAN_ENABLE
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	                    //ADC工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;	                          //启动扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                      //启动连续转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	    //转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	                //ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = ADC2_SCAN_CHANNEL_NUM;	            //扫描的通道数
	ADC_Init(ADC2, &ADC_InitStructure);	                                    //根据ADC_InitStruct中指定的参数初始化外设ADC1的寄存器   
  #else
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                      //ADC工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;                           //启动扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                     //不启动连续转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;     //转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                  //ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;                                 //顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC2, &ADC_InitStructure);                                     //根据ADC_InitStruct中指定的参数初始化外设ADC1的寄存器   
	#endif
  
  /*配置规则组各通道参数*/
	#if ADC2_SCAN_ENABLE
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 1 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 2 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_2, 3 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 4 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_4, 5 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 6 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 7 , ADC_SampleTime_239Cycles5);
	#endif
  
	ADC_Cmd(ADC2, ENABLE);	//使能指定的ADC1
	
	ADC_ResetCalibration(ADC2);	//使能复位校准  
	 
	while(ADC_GetResetCalibrationStatus(ADC2));	//等待复位校准结束
	
	ADC_StartCalibration(ADC2);	 //开启AD校准
 
	while(ADC_GetCalibrationStatus(ADC2));	 //等待校准结束
 
//	ADC_SoftwareStartConvCmd(ADC2, ENABLE);		//使能指定的ADC2的软件转换启动功能
}				  
/* 
	*函数名：ADC_SingleMode_GetAverageValue()
	*功  能：获取单次模式ADC值
	*参  数：无
	*返回值：Average
*/
uint16_t ADC_SingleMode_GetAverageValue(ADC_TypeDef* ADCx,u8 channelx)
{
  u8 t;
  u16 temp = 0;
  u16 Average = 0;
  u32 Sum_Value = 0;
  u16 Max_Value = 0;
	u16 Min_Value = 4095;
  //设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADCx, channelx, 1, ADC_SampleTime_239Cycles5);	//ADC1,ADC通道,采样时间为239.5周期	  			    

  for(t=0;t<Filter_times;t++)
	{
    ADC_SoftwareStartConvCmd(ADCx, ENABLE);		      //使能指定的ADC1的软件转换启动功能	
    
    while(!ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC));  //等待转换结束
    
    temp = ADC_GetConversionValue(ADCx);            //读取最近一次ADC1规则组的转换结果
    
    Max_Value = (Max_Value>temp)?Max_Value:temp;
		Min_Value = (Min_Value<temp)?Min_Value:temp;
		Sum_Value += temp;
	}
  /*去掉最大值和最小值再求平均值*/
	Average = (Sum_Value - Max_Value - Min_Value)/(Filter_times-2);
	return Average;	
}

/* 
	*函数名：ADC_ScanMode_GetAverageVlaue()
	*功  能：获取扫描模式ADC值
	*参  数：*zdata：数据指针
	*返回值：无
*/
/* 
	如果使能了扫描模式和DMA，则定义一个ADC_Value[][]数组来存储AD转换结果，
	共ADC1_SCAN_CHANNEL_NUM个通道，每个通道保存Filter_times组数据。
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
  
	/* 开启软件转换 */
	ADC_SoftwareStartConvCmd(ADC1 , ENABLE);
	
	/* 等待转换完成 */
	//while(ADC_GetFlagStatus(ADC1 , ADC_FLAG_EOC) == RESET);
	//对于等待转换完成，在使用实时操作系统时，可以使用延时函数交出CPU使用权，
	//从而不会形成阻塞
	delay_ms(10);

	/* 软件滤波（均值滤波） */ 
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


























