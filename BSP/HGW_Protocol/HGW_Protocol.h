/*******************************************
	*�ļ��� ��  HGW_Protocol.h
	*��   �ߣ�  WF
	*�޸�ʱ�䣺 2021.12.19
	*��   ����  v1.0
  *˵   ����  HGW���������ͨ��Э��ͷ�ļ�
*******************************************/

#ifndef _HGW_PROTOCOL_H
#define _HGW_PROTOCOL_H

#include "stm32f10x.h"

/* ��ϢȺ�������� */

/* VCU_COMMAND��������ṹ�� */
typedef struct VCU_COMMAND
{
	u8 MCU_Enable;           /* ������Ƶ�Ԫʹ�� */
	u8 Fault_Reset;          /* ���ϸ�λ */
	u8 Control_Mode;         /* ����ģʽ */
	u8 Live_Counter;         /* ѭ�������� */
	u16 Demand_Limit_High;   /* ת��/ת��ָ��ֵ���� */
  u16 Demand_Limit_Low;    /* ת��/ת��ָ��ֵ���� */
	s16 Demand_Torque;       /* Ŀ��ת�� */
	s16 Demand_Speed;        /* Ŀ��ת�� */
	
}VCU_COMMAND_t;

/* ����ģʽ */
typedef enum
{
	Speed_Control = 1,        /* ת�ٿ��� */
	Torque_Control = 2,       /* ת�ؿ��� */
	Ative_Discharge = 3       /* �����ŵ� */
	
}Control_Mode;

/* VCU_COMMAND2��������ṹ�� */
typedef struct VCU_COMMAND2
{
	u8 Brake_Pedal_Travel;      /* �ƶ�̤���г� */
	u8 Accelerate_Pedal_Travel; /* ����̤���г� */
	u8 Hand_Brake_Signal;       /* ��ɲ�ź� */
	u8 Gears_Signal;            /* ��λ�ź� */
	u8 Life_Signal;             /* �����ź� */
	
}VCU_COMMAND2_t;

/* ��λ */
typedef enum
{
	Gears_N = 0,
	Gears_R = 0x0D,
	Gears_D = 0x0E,
	Gears_P = 0x0F
}Gears;


/* MCU_STATUS1��������ṹ�� */
typedef struct MCU_STATUS1_Type
{
	s16 Motor_Speed;            /* ���ת�� */
	s16 Motor_Torque;           /* ���ת�� */
	u16 Motor_Phase_Current;    /* ����������Чֵ */
	u8 Precharge_Allow;         /* ����Ԥ��� */
	u8 Active_Discharge_Allow;  /* ����������� */
	u8 MCU_Enable_Feedback;     /* IGBTʹ��״̬���� */
	u8 Work_Mode;               /* MCU����ģʽ */
	u8 Live_Counter;            /* ѭ�������� */
	
}MCU_STATUS1_t;

/* MCU_STATUS2��������ṹ�� */
typedef struct MCU_STATUS2_Type
{
	u8 Motor_Temperature;     /* ��������¶� */
	u8 MCU_Temperature;       /* �����¶� */
	s16 Torque_Limit_High;    /* ת������ */
	s16 Torque_Limit_Low;     /* ת������ */
	u8 Fail_Grade;            /* ���ϵȼ� */
	u8 Live_Counter;          /* ѭ�������� */
	
}MCU_STATUS2_t;

/* MCU_STATUS3��������ṹ�� */
typedef struct MCU_STATUS3_Type
{
	s16 MCU_Bus_Current;      /* ������ĸ�ߵ��� */
	u16 MCU_Bus_Voltage;      /* ������ĸ�ߵ�ѹ */
	u16 Motor_Rotation_Count; /* ���ת��ѭ�������� */
	u8 Live_Counter;          /* ѭ�������� */
	
}MCU_STATUS3_t;

/* MCU_STATUS4��������ṹ�� */
typedef struct MCU_STATUS4_Type
{
	u32 bit_0_Motor_overspeed_shutdown : 1;     /* ������ٹض� */
	u32 bit_1_Motor_overtemp_shutdown : 1;      /* ���������¹ض� */
	u32 bit_2_MCU_overtemp_shutdown : 1;        /* ���������¹ض� */
	u32 bit_3_MCU_overcurrent_shutdown : 1;     /* �����������ض� */
	u32 bit_4_Motor_overvoltage_shutdown : 1;   /* ��������ѹ�ض� */
	u32 bit_5_Motor_overspeed_fault : 1;        /* ������ٹ��� */
	u32 bit_6_Motor_blocked_fault : 1;          /* �����ת���� */
	u32 bit_7_MCU_overcurrent_fault : 1;        /* �������������� */
	u32 bit_8_Motor_overtemp_fault : 1;         /* ���������¹���  */
	u32 bit_9_Motor_overtemp_warning : 1;       /* ���������¾��� */
	u32 bit_10_MCU_overtemp_fault : 1;           /* ���������¹��� */
	u32 bit_11_MCU_overtemp_warning : 1;         /* ���������¾��� */
	u32 bit_12_MCU_overvoltage_fault : 1;        /* ��������ѹ���� */
	u32 bit_13_MCU_overvoltage_warning : 1;      /* ��������ѹ���� */
	u32 bit_14_MCU_undervoltage_fault : 1;       /* ������Ƿѹ���� */
	u32 bit_15_MCU_undervoltage_warning : 1;     /* ������Ƿѹ���� */
	u32 bit_16_CAN_communication_fault : 1;      /* CAN ͨ�Ź��� */
	u32 bit_17_Low_Under_Voltage : 1;            /* ��ѹǷѹ */
	u32 bit_18_Motor_rotorlocation_falut : 1;    /* ���ת��λ�ô��������� */
	u32 bit_19_Motor_temperature1_falut : 1;     /* ��������¶ȴ����� 1 ���� */
	u32 bit_20_Motor_temperature2_falut : 1;     /* ��������¶ȴ����� 2 ���� */
	u32 bit_21_MCU_U_temperature_falut : 1;      /* ������ U ���¶ȴ��������� */
	u32 bit_22_MCU_V_temperature_falut : 1;      /* ������ V ���¶ȴ��������� */
	u32 bit_23_MCU_W_temperature_falut : 1;      /* ������ W ���¶ȴ��������� */
	u32 bit_24_MCU_U_current_falut : 1;          /* ������ U ��������������� */
	u32 bit_25_MCU_V_current_falut : 1;          /* ������ V ��������������� */
	u32 bit_26_MCU_W_current_falut : 1;          /* ������ W ��������������� */
	u32 bit_27_MCU_Bus_current_falut : 1;        /* ������ĸ�ߵ������������� */
	u32 bit_28_MCU_U_power_falut : 1;            /* ������ U �๦��ģ����� */
	u32 bit_29_MCU_V_power_falut : 1;            /* ������ V �๦��ģ����� */
	u32 bit_30_MCU_W_power_falut : 1;            /* ������ W �๦��ģ����� */
	u32 bit_31_MCU_RAM_falut : 1;                /* ������ RAM ���� */
	u8 bit_32_MCU_EEPROM_fault : 1;              /* ������ EEPROM ���� */
	u8 bit_33_Motor_shortcircuit_fault : 1;      /* ��������·���� */
	u8 bit_34_Motor_opencircuit_fault : 1;       /* ������鿪·���� */
	u8 bit_35_Active_discharge_fault : 1;        /* �����ŵ���� */
	u8 : 0;
	
	u8 Live_Counter;                      /* ѭ�������� */
	
}MCU_STATUS4_t;

/*
���ݽ�����㷽����
������ =�������� * Scale��+ Offset
������ =�������� �C Offset��/ Scale
*/


/******************************************************************************
														 �������ݴ���������
*******************************************************************************/
u8 VCU_COMMAND_SendData_Process(VCU_COMMAND_t vcu_cmd, u8 *sendbuf);
u8 VCU_COMMAND2_SendData_Process(VCU_COMMAND2_t vcu_cmd, u8 *sendbuf);
u8 VCU_COMMAD_ParaCheck(VCU_COMMAND_t vcu_cmd);
u8 VCU_COMMAD2_ParaCheck(VCU_COMMAND2_t vcu_cmd);


/******************************************************************************
														 �������ݽ�����������
*******************************************************************************/
u8 MCU_STATUS1_ParseData(MCU_STATUS1_t *mcu_sta, u8 *recivebuf);
u8 MCU_STATUS2_ParseData(MCU_STATUS2_t *mcu_sta, u8 *recivebuf);
u8 MCU_STATUS3_ParseData(MCU_STATUS3_t *mcu_sta, u8 *recivebuf);
u8 MCU_STATUS4_ParseData(MCU_STATUS4_t *mcu_sta, u8 *recivebuf);





#endif



