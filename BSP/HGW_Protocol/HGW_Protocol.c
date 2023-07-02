/*******************************************
	*文件名 ：  HGW_Protocol.c
	*作   者：  WF
	*修改时间： 2021.12.19
	*版   本：  v1.0
  *说   明：  HGW电机控制器通信协议源文件
*******************************************/

#include "HGW_Protocol.h"

/******************************************************************************
														 发送数据处理函数定义
*******************************************************************************/

/* 
	*函数名：VCU_COMMAND_SendData_Process()
	*功  能：VCU_COMMAND命令发送数据加工函数
	*作  者：WF
	*参  数：VCU_COMMAND_t vcu_cmd：命令参数结构体, u8 *sendbuf：发送数据缓存
	*返回值：返回0：表示加工完成，返回1表示参数错误
	*时  间：2021.12.19
*/
u8 VCU_COMMAND_SendData_Process(VCU_COMMAND_t vcu_cmd, u8 *sendbuf)
{
	/* 检查参数是否合法 */
	if(VCU_COMMAD_ParaCheck(vcu_cmd) == 1) /* 参数不合法 */
	{
		return 1;
	}
	
	sendbuf[0] &= 0x00;
	sendbuf[0] |= vcu_cmd.MCU_Enable;                  /* 电机控制单元使能位 */
	sendbuf[0] |= (vcu_cmd.Fault_Reset << 1);          /* 故障复位使能位 */
	sendbuf[0] |= (vcu_cmd.Control_Mode << 2);         /* 控制模式位 */
	sendbuf[0] |= (vcu_cmd.Live_Counter << 4);         /* 循环计数器 */
	sendbuf[1] = (vcu_cmd.Demand_Limit_High&0xff);     /* 转矩/转速上限低8位 */
	sendbuf[2] &= 0x00;
	sendbuf[2] |= ((vcu_cmd.Demand_Limit_High&0xf00)>>8); /* 转矩/转速上限高4位 */
	sendbuf[2] |= ((vcu_cmd.Demand_Limit_Low&0x0f)<<4);   /* 转矩/转速下限低4位 */
	sendbuf[3] &= 0x00;
	sendbuf[3] = ((vcu_cmd.Demand_Limit_Low&0xff0)>>4);   /* 转矩/转速下限高8位 */
  sendbuf[4] &= 0x00;	
	sendbuf[4] = ((vcu_cmd.Demand_Torque+5000)&0xff);     /* 目标转矩低8位 */
	sendbuf[5] &= 0x00;
	sendbuf[5] = (((vcu_cmd.Demand_Torque+5000)&0xff00)>>8); /* 目标转矩高8位 */
	sendbuf[6] &= 0x00;	
	sendbuf[6] = ((vcu_cmd.Demand_Speed+15000)&0xff);     /* 目标转速低8位 */
	sendbuf[7] &= 0x00;
	sendbuf[7] = (((vcu_cmd.Demand_Speed+15000)&0xff00)>>8); /* 目标转速高8位 */
	
	return 0;
}

/* 
	*函数名：VCU_COMMAND2_SendData_Process()
	*功  能：VCU_COMMAND2命令发送数据加工函数
	*作  者：WF
	*参  数：VCU_COMMAND_t vcu_cmd：命令参数结构体, u8 *sendbuf：发送数据缓存
	*返回值：返回0：表示加工完成，返回1表示参数错误
	*时  间：2021.12.19
*/
u8 VCU_COMMAND2_SendData_Process(VCU_COMMAND2_t vcu_cmd, u8 *sendbuf)
{
	/* 检查参数是否合法 */
	if(VCU_COMMAD2_ParaCheck(vcu_cmd) == 1)
	{
		return 1;
	}
	
	sendbuf[0] &= 0x00;
	sendbuf[0] = vcu_cmd.Brake_Pedal_Travel;        /* 制动踏板行程值 */
	sendbuf[1] &= 0x00;
	sendbuf[1] = vcu_cmd.Accelerate_Pedal_Travel;   /* 加速踏板行程值 */
	sendbuf[2] &= 0x00;
	sendbuf[2] = vcu_cmd.Hand_Brake_Signal;         /* 手刹信号 */
	sendbuf[3] &= 0x00;
	sendbuf[3] |= vcu_cmd.Gears_Signal;             /* 挡位信号 */
	sendbuf[4] &= 0x00;
	sendbuf[5] &= 0x00;
	sendbuf[6] &= 0x00;
	sendbuf[7] &= 0x00;
	sendbuf[7] = vcu_cmd.Life_Signal;               /* 循环计数器 */
	
	return 0;
}


/* VCU_COMMAD命令参数有效性检查函数 */
u8 VCU_COMMAD_ParaCheck(VCU_COMMAND_t vcu_cmd)
{
	/* 检查参数取值范围 */
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

/* VCU_COMMAD2命令参数有效性检查函数 */
u8 VCU_COMMAD2_ParaCheck(VCU_COMMAND2_t vcu_cmd)
{
	/* 检查参数的取值范围 */
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
														 接收数据解析函数定义
*******************************************************************************/

/* 
	*函数名：MCU_STATUS1_ParseData()
	*功  能：MCU_STATUS1命令接收数据解析函数
	*作  者：WF
	*参  数：MCU_STATUS1_t *mcu_sta：命令参数结构体, u8 *recivebuf：接收数据缓存
	*返回值：返回0：表示加工完成，返回1表示参数错误
	*时  间：2021.12.19
*/
u8 MCU_STATUS1_ParseData(MCU_STATUS1_t *mcu_sta, u8 *recivebuf)
{
	s16 tempdata = 0;
	
	/* 解析转速数据 */
	tempdata = (recivebuf[1]*256 + recivebuf[0])*1 - 15000;
	
//	if(tempdata < -15000 || tempdata > 15000) /* 电机转速数据不合法 */
//	{
//		return 1;
//	}
//	else
//	{
		mcu_sta->Motor_Speed = tempdata;
//	}
	
	/* 解析转矩数据 */
	tempdata = (recivebuf[3]*256 + recivebuf[2])*1 - 5000;
	
//	if(tempdata < -5000 || tempdata > 5000) /* 电机转矩数据不合法 */
//	{
//		return 1;
//	}
//	else
//	{
		mcu_sta->Motor_Torque = tempdata;
//	}
	
	/* 解析相电流数据 */
	tempdata = (recivebuf[5]*256 + recivebuf[4])*0.1 + 0;
	
//	if(tempdata > 1000) /* 相电流数据不合法 */
//	{
//		return 1;
//	}
//	else
//	{
		mcu_sta->Motor_Phase_Current = tempdata;
//	}
	
	/* 解析允许预充电位 */
	mcu_sta->Precharge_Allow = ((recivebuf[6]&0x10)>>4);
	/* 解析允许主动放电 */
	mcu_sta->Active_Discharge_Allow = ((recivebuf[6]&0x20)>>5);
	/* 解析IGBT 使能状态反馈 */
	mcu_sta->MCU_Enable_Feedback = ((recivebuf[6]&0x40)>>6);
	
	/* 解析电机工作模式 */
//	if((recivebuf[7]&0x0f) > 8) /* 电机工作模式不合法 */
//	{
//		return 1;
//	}
//	else
//	{
		mcu_sta->Work_Mode = (recivebuf[7]&0x0f);
//	}
	
	/* 解析循环计数器 */
	mcu_sta->Live_Counter = ((recivebuf[7]&0xf0)>>4);
	
	return 0;
}


/* 
	*函数名：MCU_STATUS2_ParseData()
	*功  能：MCU_STATUS2命令接收数据解析函数
	*作  者：WF
	*参  数：MCU_STATUS2_t *mcu_sta：命令参数结构体, u8 *recivebuf：接收数据缓存
	*返回值：返回0：表示加工完成，返回1表示参数错误
	*时  间：2021.12.19
*/
u8 MCU_STATUS2_ParseData(MCU_STATUS2_t *mcu_sta, u8 *recivebuf)
{
	s16 tempdata = 0;
	
	mcu_sta->Motor_Temperature = recivebuf[0]*1 - 40;
	mcu_sta->MCU_Temperature = recivebuf[1]*1 - 40;
	
	/* 解析转矩上限 */
	tempdata = (recivebuf[3]*256 + recivebuf[2])*1 - 0;
	
//	if(tempdata > 5000 || tempdata < 0) /* 电机转矩上限数据不合法 */
//	{
//		return 1;
//	}
//	else
//	{
		mcu_sta->Torque_Limit_High = tempdata;
//	}
	
	/* 解析转矩下限 */
	tempdata = (recivebuf[5]*256 + recivebuf[4])*1 - 0;
	
//	if(tempdata > 0 || tempdata < -5000) /* 电机转矩下限数据不合法 */
//	{
//		return 1;
//	}
//	else
//	{
		mcu_sta->Torque_Limit_Low = tempdata;
//	}
	
	/* 解析故障等级 */
	if((recivebuf[6]&0x0f) > 4) /* 故障等级不合法 */
//	{
//		return 1;
//	}
//	else
//	{
		mcu_sta->Fail_Grade = (recivebuf[6]&0x0f);
//	}
	
	/* 解析循环计数器 */
	mcu_sta->Live_Counter = ((recivebuf[7]&0xf0)>>4);
	
	return 0;
}

/* 
	*函数名：MCU_STATUS3_ParseData()
	*功  能：MCU_STATUS3命令接收数据解析函数
	*作  者：WF
	*参  数：MCU_STATUS3_t *mcu_sta：命令参数结构体, u8 *recivebuf：接收数据缓存
	*返回值：返回0：表示加工完成，返回1表示参数错误
	*时  间：2021.12.19
*/
u8 MCU_STATUS3_ParseData(MCU_STATUS3_t *mcu_sta, u8 *recivebuf)
{
	s16 tempdata = 0;
	
	/* 解析控制器母线电流 */
	tempdata = (recivebuf[3]*256 + recivebuf[2])*0.1 - 1000;
	
//	if(tempdata < -1000 || tempdata > 1000)
//	{
//		return 1;
//	}
//	else
//	{
		mcu_sta->MCU_Bus_Current = tempdata;
//	}
	
	/* 解析控制器母线电压 */
	tempdata = (recivebuf[5]*256 + recivebuf[4])*0.1 + 0;
	
//	if(tempdata > 1000)
//	{
//		return 1;
//	}
//	else
//	{
		mcu_sta->MCU_Bus_Voltage = tempdata;
//	}
	
	/* 解析电机转数循环计数 */
	mcu_sta->Motor_Rotation_Count = ((recivebuf[7]&0x0f)*256 + recivebuf[6])*1 + 0;
	/* 解析循环计数器 */
	mcu_sta->Live_Counter = ((recivebuf[7]&0xf0)>>4);
	
	return 0;
}


/* 
	*函数名：MCU_STATUS4_ParseData()
	*功  能：MCU_STATUS4命令接收数据解析函数
	*作  者：WF
	*参  数：MCU_STATUS4_t *mcu_sta：命令参数结构体, u8 *recivebuf：接收数据缓存
	*返回值：返回0：表示加工完成，返回1表示参数错误
	*时  间：2021.12.19
*/
u8 MCU_STATUS4_ParseData(MCU_STATUS4_t *mcu_sta, u8 *recivebuf)
{
	/* 解析关断、故障、警告状态位 */
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


