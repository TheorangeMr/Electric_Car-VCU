/*******************************************
	*�ļ��� ��  HGW_Protocol.c
	*��   �ߣ�  WF
	*�޸�ʱ�䣺 2021.12.19
	*��   ����  v1.0
  *˵   ����  HGW���������ͨ��Э��Դ�ļ�
*******************************************/

#include "HGW_Protocol.h"

/******************************************************************************
														 �������ݴ���������
*******************************************************************************/

/* 
	*��������VCU_COMMAND_SendData_Process()
	*��  �ܣ�VCU_COMMAND��������ݼӹ�����
	*��  �ߣ�WF
	*��  ����VCU_COMMAND_t vcu_cmd����������ṹ��, u8 *sendbuf���������ݻ���
	*����ֵ������0����ʾ�ӹ���ɣ�����1��ʾ��������
	*ʱ  �䣺2021.12.19
*/
u8 VCU_COMMAND_SendData_Process(VCU_COMMAND_t vcu_cmd, u8 *sendbuf)
{
	/* �������Ƿ�Ϸ� */
	if(VCU_COMMAD_ParaCheck(vcu_cmd) == 1) /* �������Ϸ� */
	{
		return 1;
	}
	
	sendbuf[0] &= 0x00;
	sendbuf[0] |= vcu_cmd.MCU_Enable;                  /* ������Ƶ�Ԫʹ��λ */
	sendbuf[0] |= (vcu_cmd.Fault_Reset << 1);          /* ���ϸ�λʹ��λ */
	sendbuf[0] |= (vcu_cmd.Control_Mode << 2);         /* ����ģʽλ */
	sendbuf[0] |= (vcu_cmd.Live_Counter << 4);         /* ѭ�������� */
	sendbuf[1] = (vcu_cmd.Demand_Limit_High&0xff);     /* ת��/ת�����޵�8λ */
	sendbuf[2] &= 0x00;
	sendbuf[2] |= ((vcu_cmd.Demand_Limit_High&0xf00)>>8); /* ת��/ת�����޸�4λ */
	sendbuf[2] |= ((vcu_cmd.Demand_Limit_Low&0x0f)<<4);   /* ת��/ת�����޵�4λ */
	sendbuf[3] &= 0x00;
	sendbuf[3] = ((vcu_cmd.Demand_Limit_Low&0xff0)>>4);   /* ת��/ת�����޸�8λ */
  sendbuf[4] &= 0x00;	
	sendbuf[4] = ((vcu_cmd.Demand_Torque+5000)&0xff);     /* Ŀ��ת�ص�8λ */
	sendbuf[5] &= 0x00;
	sendbuf[5] = (((vcu_cmd.Demand_Torque+5000)&0xff00)>>8); /* Ŀ��ת�ظ�8λ */
	sendbuf[6] &= 0x00;	
	sendbuf[6] = ((vcu_cmd.Demand_Speed+15000)&0xff);     /* Ŀ��ת�ٵ�8λ */
	sendbuf[7] &= 0x00;
	sendbuf[7] = (((vcu_cmd.Demand_Speed+15000)&0xff00)>>8); /* Ŀ��ת�ٸ�8λ */
	
	return 0;
}

/* 
	*��������VCU_COMMAND2_SendData_Process()
	*��  �ܣ�VCU_COMMAND2��������ݼӹ�����
	*��  �ߣ�WF
	*��  ����VCU_COMMAND_t vcu_cmd����������ṹ��, u8 *sendbuf���������ݻ���
	*����ֵ������0����ʾ�ӹ���ɣ�����1��ʾ��������
	*ʱ  �䣺2021.12.19
*/
u8 VCU_COMMAND2_SendData_Process(VCU_COMMAND2_t vcu_cmd, u8 *sendbuf)
{
	/* �������Ƿ�Ϸ� */
	if(VCU_COMMAD2_ParaCheck(vcu_cmd) == 1)
	{
		return 1;
	}
	
	sendbuf[0] &= 0x00;
	sendbuf[0] = vcu_cmd.Brake_Pedal_Travel;        /* �ƶ�̤���г�ֵ */
	sendbuf[1] &= 0x00;
	sendbuf[1] = vcu_cmd.Accelerate_Pedal_Travel;   /* ����̤���г�ֵ */
	sendbuf[2] &= 0x00;
	sendbuf[2] = vcu_cmd.Hand_Brake_Signal;         /* ��ɲ�ź� */
	sendbuf[3] &= 0x00;
	sendbuf[3] |= vcu_cmd.Gears_Signal;             /* ��λ�ź� */
	sendbuf[4] &= 0x00;
	sendbuf[5] &= 0x00;
	sendbuf[6] &= 0x00;
	sendbuf[7] &= 0x00;
	sendbuf[7] = vcu_cmd.Life_Signal;               /* ѭ�������� */
	
	return 0;
}


/* VCU_COMMAD���������Ч�Լ�麯�� */
u8 VCU_COMMAD_ParaCheck(VCU_COMMAND_t vcu_cmd)
{
	/* ������ȡֵ��Χ */
	if(vcu_cmd.MCU_Enable > 1 || vcu_cmd.Fault_Reset > 1 || vcu_cmd.Control_Mode > 3 || vcu_cmd.Control_Mode < 1 ||
		 vcu_cmd.Live_Counter > 15 || vcu_cmd.Demand_Limit_High > 4095 || vcu_cmd.Demand_Limit_Low > 4095 ||
	   vcu_cmd.Demand_Torque  > 5000 || vcu_cmd.Demand_Torque < -5000 ||vcu_cmd.Demand_Speed > 15000 ||
	   vcu_cmd.Demand_Speed < -15000)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/* VCU_COMMAD2���������Ч�Լ�麯�� */
u8 VCU_COMMAD2_ParaCheck(VCU_COMMAND2_t vcu_cmd)
{
	/* ��������ȡֵ��Χ */
	if(vcu_cmd.Accelerate_Pedal_Travel > 100 || vcu_cmd.Brake_Pedal_Travel > 100 ||
		 vcu_cmd.Hand_Brake_Signal > 1 || vcu_cmd.Life_Signal > 255)
	{
		return 1;
	}
	else
	{
		if(vcu_cmd.Gears_Signal == Gears_N || vcu_cmd.Gears_Signal == Gears_R ||
			 vcu_cmd.Gears_Signal == Gears_D || vcu_cmd.Gears_Signal == Gears_P)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
}


/******************************************************************************
														 �������ݽ�����������
*******************************************************************************/

/* 
	*��������MCU_STATUS1_ParseData()
	*��  �ܣ�MCU_STATUS1����������ݽ�������
	*��  �ߣ�WF
	*��  ����MCU_STATUS1_t *mcu_sta����������ṹ��, u8 *recivebuf���������ݻ���
	*����ֵ������0����ʾ�ӹ���ɣ�����1��ʾ��������
	*ʱ  �䣺2021.12.19
*/
u8 MCU_STATUS1_ParseData(MCU_STATUS1_t *mcu_sta, u8 *recivebuf)
{
	s16 tempdata = 0;
	
	/* ����ת������ */
	tempdata = (recivebuf[1]*256 + recivebuf[0])*1 - 15000;
	
//	if(tempdata < -15000 || tempdata > 15000) /* ���ת�����ݲ��Ϸ� */
//	{
//		return 1;
//	}
//	else
//	{
		mcu_sta->Motor_Speed = tempdata;
//	}
	
	/* ����ת������ */
	tempdata = (recivebuf[3]*256 + recivebuf[2])*1 - 5000;
	
//	if(tempdata < -5000 || tempdata > 5000) /* ���ת�����ݲ��Ϸ� */
//	{
//		return 1;
//	}
//	else
//	{
		mcu_sta->Motor_Torque = tempdata;
//	}
	
	/* ������������� */
	tempdata = (recivebuf[5]*256 + recivebuf[4])*0.1 + 0;
	
//	if(tempdata > 1000) /* ��������ݲ��Ϸ� */
//	{
//		return 1;
//	}
//	else
//	{
		mcu_sta->Motor_Phase_Current = tempdata;
//	}
	
	/* ��������Ԥ���λ */
	mcu_sta->Precharge_Allow = ((recivebuf[6]&0x10)>>4);
	/* �������������ŵ� */
	mcu_sta->Active_Discharge_Allow = ((recivebuf[6]&0x20)>>5);
	/* ����IGBT ʹ��״̬���� */
	mcu_sta->MCU_Enable_Feedback = ((recivebuf[6]&0x40)>>6);
	
	/* �����������ģʽ */
//	if((recivebuf[7]&0x0f) > 8) /* �������ģʽ���Ϸ� */
//	{
//		return 1;
//	}
//	else
//	{
		mcu_sta->Work_Mode = (recivebuf[7]&0x0f);
//	}
	
	/* ����ѭ�������� */
	mcu_sta->Live_Counter = ((recivebuf[7]&0xf0)>>4);
	
	return 0;
}


/* 
	*��������MCU_STATUS2_ParseData()
	*��  �ܣ�MCU_STATUS2����������ݽ�������
	*��  �ߣ�WF
	*��  ����MCU_STATUS2_t *mcu_sta����������ṹ��, u8 *recivebuf���������ݻ���
	*����ֵ������0����ʾ�ӹ���ɣ�����1��ʾ��������
	*ʱ  �䣺2021.12.19
*/
u8 MCU_STATUS2_ParseData(MCU_STATUS2_t *mcu_sta, u8 *recivebuf)
{
	s16 tempdata = 0;
	
	mcu_sta->Motor_Temperature = recivebuf[0]*1 - 40;
	mcu_sta->MCU_Temperature = recivebuf[1]*1 - 40;
	
	/* ����ת������ */
	tempdata = (recivebuf[3]*256 + recivebuf[2])*1 - 0;
	
//	if(tempdata > 5000 || tempdata < 0) /* ���ת���������ݲ��Ϸ� */
//	{
//		return 1;
//	}
//	else
//	{
		mcu_sta->Torque_Limit_High = tempdata;
//	}
	
	/* ����ת������ */
	tempdata = (recivebuf[5]*256 + recivebuf[4])*1 - 0;
	
//	if(tempdata > 0 || tempdata < -5000) /* ���ת���������ݲ��Ϸ� */
//	{
//		return 1;
//	}
//	else
//	{
		mcu_sta->Torque_Limit_Low = tempdata;
//	}
	
	/* �������ϵȼ� */
	if((recivebuf[6]&0x0f) > 4) /* ���ϵȼ����Ϸ� */
//	{
//		return 1;
//	}
//	else
//	{
		mcu_sta->Fail_Grade = (recivebuf[6]&0x0f);
//	}
	
	/* ����ѭ�������� */
	mcu_sta->Live_Counter = ((recivebuf[7]&0xf0)>>4);
	
	return 0;
}

/* 
	*��������MCU_STATUS3_ParseData()
	*��  �ܣ�MCU_STATUS3����������ݽ�������
	*��  �ߣ�WF
	*��  ����MCU_STATUS3_t *mcu_sta����������ṹ��, u8 *recivebuf���������ݻ���
	*����ֵ������0����ʾ�ӹ���ɣ�����1��ʾ��������
	*ʱ  �䣺2021.12.19
*/
u8 MCU_STATUS3_ParseData(MCU_STATUS3_t *mcu_sta, u8 *recivebuf)
{
	s16 tempdata = 0;
	
	/* ����������ĸ�ߵ��� */
	tempdata = (recivebuf[3]*256 + recivebuf[2])*0.1 - 1000;
	
//	if(tempdata < -1000 || tempdata > 1000)
//	{
//		return 1;
//	}
//	else
//	{
		mcu_sta->MCU_Bus_Current = tempdata;
//	}
	
	/* ����������ĸ�ߵ�ѹ */
	tempdata = (recivebuf[5]*256 + recivebuf[4])*0.1 + 0;
	
//	if(tempdata > 1000)
//	{
//		return 1;
//	}
//	else
//	{
		mcu_sta->MCU_Bus_Voltage = tempdata;
//	}
	
	/* �������ת��ѭ������ */
	mcu_sta->Motor_Rotation_Count = ((recivebuf[7]&0x0f)*256 + recivebuf[6])*1 + 0;
	/* ����ѭ�������� */
	mcu_sta->Live_Counter = ((recivebuf[7]&0xf0)>>4);
	
	return 0;
}


/* 
	*��������MCU_STATUS4_ParseData()
	*��  �ܣ�MCU_STATUS4����������ݽ�������
	*��  �ߣ�WF
	*��  ����MCU_STATUS4_t *mcu_sta����������ṹ��, u8 *recivebuf���������ݻ���
	*����ֵ������0����ʾ�ӹ���ɣ�����1��ʾ��������
	*ʱ  �䣺2021.12.19
*/
u8 MCU_STATUS4_ParseData(MCU_STATUS4_t *mcu_sta, u8 *recivebuf)
{
	/* �����ضϡ����ϡ�����״̬λ */
	mcu_sta->bit_0_Motor_overspeed_shutdown = ((recivebuf[0]&0x01)>>0);
	mcu_sta->bit_1_Motor_overtemp_shutdown = ((recivebuf[0]&0x02)>>1);
	mcu_sta->bit_2_MCU_overtemp_shutdown = ((recivebuf[0]&0x04)>>2);
	mcu_sta->bit_3_MCU_overcurrent_shutdown = ((recivebuf[0]&0x08)>>3);
	mcu_sta->bit_4_Motor_overvoltage_shutdown = ((recivebuf[0]&0x10)>>4);
	mcu_sta->bit_5_Motor_overspeed_fault = ((recivebuf[0]&0x20)>>5);
	mcu_sta->bit_6_Motor_blocked_fault = ((recivebuf[0]&0x40)>>6);
	mcu_sta->bit_7_MCU_overcurrent_fault = ((recivebuf[0]&0x80)>>7);
	
	mcu_sta->bit_8_Motor_overtemp_fault = ((recivebuf[1]&0x01)>>0);
	mcu_sta->bit_9_Motor_overtemp_warning = ((recivebuf[1]&0x02)>>1);
	mcu_sta->bit_10_MCU_overtemp_fault = ((recivebuf[1]&0x04)>>2);
	mcu_sta->bit_11_MCU_overtemp_warning = ((recivebuf[1]&0x08)>>3);
	mcu_sta->bit_12_MCU_overvoltage_fault = ((recivebuf[1]&0x10)>>4);
	mcu_sta->bit_13_MCU_overvoltage_warning = ((recivebuf[1]&0x20)>>5);
	mcu_sta->bit_14_MCU_undervoltage_fault = ((recivebuf[1]&0x40)>>6);
	mcu_sta->bit_15_MCU_undervoltage_warning = ((recivebuf[1]&0x80)>>7);
	
	mcu_sta->bit_16_CAN_communication_fault = ((recivebuf[2]&0x01)>>0);
	mcu_sta->bit_17_Low_Under_Voltage = ((recivebuf[2]&0x02)>>1);
	mcu_sta->bit_18_Motor_rotorlocation_falut = ((recivebuf[2]&0x04)>>2);
	mcu_sta->bit_19_Motor_temperature1_falut = ((recivebuf[2]&0x08)>>3);
	mcu_sta->bit_20_Motor_temperature2_falut = ((recivebuf[2]&0x10)>>4);
	mcu_sta->bit_21_MCU_U_temperature_falut = ((recivebuf[2]&0x20)>>5);
	mcu_sta->bit_22_MCU_V_temperature_falut = ((recivebuf[2]&0x40)>>6);
	mcu_sta->bit_23_MCU_W_temperature_falut = ((recivebuf[2]&0x80)>>7);
	
	mcu_sta->bit_24_MCU_U_current_falut = ((recivebuf[3]&0x01)>>0);
	mcu_sta->bit_25_MCU_V_current_falut = ((recivebuf[3]&0x02)>>1);
	mcu_sta->bit_26_MCU_W_current_falut = ((recivebuf[3]&0x04)>>2);
	mcu_sta->bit_27_MCU_Bus_current_falut = ((recivebuf[3]&0x08)>>3);
	mcu_sta->bit_28_MCU_U_power_falut = ((recivebuf[3]&0x10)>>4);
	mcu_sta->bit_29_MCU_V_power_falut = ((recivebuf[3]&0x20)>>5);
	mcu_sta->bit_30_MCU_W_power_falut = ((recivebuf[3]&0x40)>>6);
	mcu_sta->bit_31_MCU_RAM_falut = ((recivebuf[3]&0x80)>>7);
	
	mcu_sta->bit_32_MCU_EEPROM_fault = ((recivebuf[4]&0x01)>>0);
	mcu_sta->bit_33_Motor_shortcircuit_fault = ((recivebuf[4]&0x02)>>1);
	mcu_sta->bit_34_Motor_opencircuit_fault = ((recivebuf[4]&0x04)>>2);
	mcu_sta->bit_35_Active_discharge_fault = ((recivebuf[4]&0x08)>>3);
	
	mcu_sta->Live_Counter = ((recivebuf[7]&0xf0)>>4);
	
	return 0;
}


