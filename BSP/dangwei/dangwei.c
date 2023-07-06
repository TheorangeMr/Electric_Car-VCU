#include "dangwei.h"


void dangwei_gpio_init(void)
{
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOE, ENABLE);

    //GPIOA8   待驶开关
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 			
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);           

    //GPIOE5  前进挡
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;					//上拉输入
    GPIO_Init(GPIOE, &GPIO_InitStructure);
	
    //GPIOE6  后退挡
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;					//上拉输入
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}


uint8_t Key_scan(void)
{
	static uint8_t key_flag = 0;
	if(key_flag == 0)
	{
		key_flag = 1;
		vTaskDelay(200);
		if(KA8 == 0||KE5 == 0||KE6 == 0)
		{
			if(KA8 == 0){return 1;}
				else if(KE5 == 0){return 2;}
					else if(KE6 == 0){return 3;}
		}
	}
	else if(KA8 == 1&&KE5 == 1&&KE6 == 1){key_flag = 0;}
	return 0xff;
}

