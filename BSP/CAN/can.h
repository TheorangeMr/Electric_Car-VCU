#ifndef __CAN_H
#define __CAN_H	     							    
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"

#define EVENTBIT_1	(1<<1)				//MCU_STATUS1ָ���¼�λ
#define EVENTBIT_2	(1<<2)				//MCU_STATUS2ָ���¼�λ
#define EVENTBIT_3	(1<<3)				//MCU_STATUS3ָ���¼�λ
#define EVENTBIT_4	(1<<4)				//MCU_STATUS4ָ���¼�λ
#define EVENTBIT_5	(1<<5)				//DBG_STATUSָ���¼�λ

//CAN����RX0�ж�ʹ��
#define CAN_RX0_INT_ENABLE	1		//0,��ʹ��; 1,ʹ��.

/************************************************************
								CAN������ź�ʱ�Ӻ궨��
************************************************************/
#define CANx                 CAN1
#define CAN_CLK              RCC_APB1Periph_CAN1
#define CAN_RX_IRQ					 USB_LP_CAN1_RX0_IRQn
#define CAN_RX_IRQHandler    USB_LP_CAN1_RX0_IRQHandler

/* CAN RX����*/
#define CAN_RX_PORT          GPIOA
#define CAN_RX_CLK           RCC_APB2Periph_GPIOA
#define CAN_RX_PIN           GPIO_Pin_11

/* CAN TX���� */
#define CAN_TX_PORT          GPIOA
#define CAN_TX_CLK           (RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA)
#define CAN_TX_PIN           GPIO_Pin_12

/************************************************************
	                    CAN���⺯���ӿ�
************************************************************/
void CAN_Config(void);
u8 Can_Send_Msg(u32 Ext_ID,u8* msg,u8 len);     //��������
void Wit_Can_Send_Msg(uint8_t ucStdId, uint8_t* msg, uint32_t len);
u8 Can_Receive_Msg(u8 *msg,u8 len);            //��������

/* ���䷵�ر�־ */
#define FAILED (0) //����ʧ��
#define PASSED (1) //����ɹ�

/* ��ϢȺ���� */
#define VCU_Command  ((u32)0x0CFF08EF) //��ʶ��ID��0x0CFF08EF   ���ȣ�8   ���ڣ�10ms
#define VCU_Command2 ((u32)0x18FF0101) //��ʶ��ID��0x18FF0101   ���ȣ�8   ���ڣ�50ms
#define MCU_Status1  ((u32)0x0CFF0008) //��ʶ��ID��0x0CFF0008   ���ȣ�8   ���ڣ�10ms
#define MCU_Status2  ((u32)0x0CFF0108) //��ʶ��ID��0x0CFF0108   ���ȣ�8   ���ڣ�20ms
#define MCU_Status3  ((u32)0x0CFF0208) //��ʶ��ID��0x0CFF0208   ���ȣ�8   ���ڣ�20ms
#define MCU_Status4  ((u32)0x0CFF0308) //��ʶ��ID��0x0CFF0308   ���ȣ�8   ���ڣ�50ms
#define DBG_Command  ((u32)0x0CFF08FF) //��ʶ��ID��0x0CFF08FF   ���ȣ�8   ���ڣ�50ms
#define DBG_Status   ((u32)0x18EFFFF5) //��ʶ��ID��0x18EFFFF5   ���ȣ�8   ���ڣ�50ms

#define Wit_dat   ((u32)0x50) //��ʶ��ID��0x50    (��׼֡)


#define VCU_Command_flag   (1)
#define VCU_Command2_flag  (2)
#define MCU_Status1_flag   (3)
#define MCU_Status2_flag   (4)
#define MCU_Status3_flag   (5)
#define MCU_Status4_flag   (6)
#define DBG_Command_flag   (7)
#define DBG_Status_flag    (8)

#endif

















