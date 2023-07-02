/*******************************************
	*�ļ��� ��  usart
	*��   �ߣ�  �޳�
	*�޸�ʱ�䣺 2022.10.5
	*��   ����  1.0
*******************************************/
#include "usart.h"

void uart_init(u32 bound)
{
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK | DEBUG_USART_GPIO_CLK, ENABLE);    //ʹ��USART1��GPIOAʱ��

    //USART1_TX   GPIOA9
    GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    //�����������
    GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);//��ʼ��GPIOA9

    //USART1_RX      GPIOA10��ʼ��
    GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;//PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);//��ʼ��GPIOA10

    //Usart1 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1 ; //��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;            //IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);    //����ָ���Ĳ�����ʼ��VIC�Ĵ���

    //USART ��ʼ������

    USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    //�շ�ģʽ

    USART_Init(DEBUG_USARTx, &USART_InitStructure); //��ʼ������1
    USART_Cmd(DEBUG_USARTx, ENABLE);                    //ʹ�ܴ���1

}

//�ض���fputc����
int fputc(int ch, FILE *f)
{
    while((USART1->SR & 0X40) == 0); //ѭ������,ֱ���������

    USART1->DR = (u8) ch;
    return ch;
}


void usart1_send_data(u8 *data, u32 size)
{
    for(u32 i = 0; i < size; i++)
    {
        while((USART1->SR & 0X40) == 0);

        USART1->DR = data[i];
    }
}
