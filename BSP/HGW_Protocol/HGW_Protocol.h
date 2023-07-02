/*******************************************
	*文件名 ：  HGW_Protocol.h
	*作   者：  WF
	*修改时间： 2021.12.19
	*版   本：  v1.0
  *说   明：  HGW电机控制器通信协议头文件
*******************************************/

#ifndef _HGW_PROTOCOL_H
#define _HGW_PROTOCOL_H

#include "stm32f10x.h"

/* 消息群参数定义 */

/* VCU_COMMAND命令参数结构体 */
typedef struct VCU_COMMAND
{
	u8 MCU_Enable;           /* 电机控制单元使能 */
	u8 Fault_Reset;          /* 故障复位 */
	u8 Control_Mode;         /* 控制模式 */
	u8 Live_Counter;         /* 循环计数器 */
	u16 Demand_Limit_High;   /* 转矩/转速指令值上限 */
  u16 Demand_Limit_Low;    /* 转矩/转速指令值下限 */
	s16 Demand_Torque;       /* 目标转矩 */
	s16 Demand_Speed;        /* 目标转速 */
	
}VCU_COMMAND_t;

/* 控制模式 */
typedef enum
{
	Speed_Control = 1,        /* 转速控制 */
	Torque_Control = 2,       /* 转矩控制 */
	Ative_Discharge = 3       /* 主动放电 */
	
}Control_Mode;

/* VCU_COMMAND2命令参数结构体 */
typedef struct VCU_COMMAND2
{
	u8 Brake_Pedal_Travel;      /* 制动踏板行程 */
	u8 Accelerate_Pedal_Travel; /* 加速踏板行程 */
	u8 Hand_Brake_Signal;       /* 手刹信号 */
	u8 Gears_Signal;            /* 挡位信号 */
	u8 Life_Signal;             /* 生命信号 */
	
}VCU_COMMAND2_t;

/* 挡位 */
typedef enum
{
	Gears_N = 0,
	Gears_R = 0x0D,
	Gears_D = 0x0E,
	Gears_P = 0x0F
}Gears;


/* MCU_STATUS1命令参数结构体 */
typedef struct MCU_STATUS1_Type
{
	s16 Motor_Speed;            /* 电机转速 */
	s16 Motor_Torque;           /* 电机转矩 */
	u16 Motor_Phase_Current;    /* 电机相电流有效值 */
	u8 Precharge_Allow;         /* 允许预充电 */
	u8 Active_Discharge_Allow;  /* 允许主动充电 */
	u8 MCU_Enable_Feedback;     /* IGBT使能状态反馈 */
	u8 Work_Mode;               /* MCU工作模式 */
	u8 Live_Counter;            /* 循环计数器 */
	
}MCU_STATUS1_t;

/* MCU_STATUS2命令参数结构体 */
typedef struct MCU_STATUS2_Type
{
	u8 Motor_Temperature;     /* 电机绕组温度 */
	u8 MCU_Temperature;       /* 控制温度 */
	s16 Torque_Limit_High;    /* 转矩上限 */
	s16 Torque_Limit_Low;     /* 转矩下线 */
	u8 Fail_Grade;            /* 故障等级 */
	u8 Live_Counter;          /* 循环计数器 */
	
}MCU_STATUS2_t;

/* MCU_STATUS3命令参数结构体 */
typedef struct MCU_STATUS3_Type
{
	s16 MCU_Bus_Current;      /* 控制器母线电流 */
	u16 MCU_Bus_Voltage;      /* 控制器母线电压 */
	u16 Motor_Rotation_Count; /* 电机转速循环计数器 */
	u8 Live_Counter;          /* 循环计数器 */
	
}MCU_STATUS3_t;

/* MCU_STATUS4命令参数结构体 */
typedef struct MCU_STATUS4_Type
{
	u32 bit_0_Motor_overspeed_shutdown : 1;     /* 电机超速关断 */
	u32 bit_1_Motor_overtemp_shutdown : 1;      /* 电机绕组过温关断 */
	u32 bit_2_MCU_overtemp_shutdown : 1;        /* 控制器过温关断 */
	u32 bit_3_MCU_overcurrent_shutdown : 1;     /* 控制器过流关断 */
	u32 bit_4_Motor_overvoltage_shutdown : 1;   /* 控制器过压关断 */
	u32 bit_5_Motor_overspeed_fault : 1;        /* 电机超速故障 */
	u32 bit_6_Motor_blocked_fault : 1;          /* 电机堵转故障 */
	u32 bit_7_MCU_overcurrent_fault : 1;        /* 控制器过流故障 */
	u32 bit_8_Motor_overtemp_fault : 1;         /* 电机绕组过温故障  */
	u32 bit_9_Motor_overtemp_warning : 1;       /* 电机绕组过温警告 */
	u32 bit_10_MCU_overtemp_fault : 1;           /* 控制器过温故障 */
	u32 bit_11_MCU_overtemp_warning : 1;         /* 控制器过温警告 */
	u32 bit_12_MCU_overvoltage_fault : 1;        /* 控制器过压故障 */
	u32 bit_13_MCU_overvoltage_warning : 1;      /* 控制器过压警告 */
	u32 bit_14_MCU_undervoltage_fault : 1;       /* 控制器欠压故障 */
	u32 bit_15_MCU_undervoltage_warning : 1;     /* 控制器欠压警告 */
	u32 bit_16_CAN_communication_fault : 1;      /* CAN 通信故障 */
	u32 bit_17_Low_Under_Voltage : 1;            /* 低压欠压 */
	u32 bit_18_Motor_rotorlocation_falut : 1;    /* 电机转子位置传感器故障 */
	u32 bit_19_Motor_temperature1_falut : 1;     /* 电机绕组温度传感器 1 故障 */
	u32 bit_20_Motor_temperature2_falut : 1;     /* 电机绕组温度传感器 2 故障 */
	u32 bit_21_MCU_U_temperature_falut : 1;      /* 控制器 U 相温度传感器故障 */
	u32 bit_22_MCU_V_temperature_falut : 1;      /* 控制器 V 相温度传感器故障 */
	u32 bit_23_MCU_W_temperature_falut : 1;      /* 控制器 W 相温度传感器故障 */
	u32 bit_24_MCU_U_current_falut : 1;          /* 控制器 U 相电流传感器故障 */
	u32 bit_25_MCU_V_current_falut : 1;          /* 控制器 V 相电流传感器故障 */
	u32 bit_26_MCU_W_current_falut : 1;          /* 控制器 W 相电流传感器故障 */
	u32 bit_27_MCU_Bus_current_falut : 1;        /* 控制器母线电流传感器故障 */
	u32 bit_28_MCU_U_power_falut : 1;            /* 控制器 U 相功率模块故障 */
	u32 bit_29_MCU_V_power_falut : 1;            /* 控制器 V 相功率模块故障 */
	u32 bit_30_MCU_W_power_falut : 1;            /* 控制器 W 相功率模块故障 */
	u32 bit_31_MCU_RAM_falut : 1;                /* 控制器 RAM 故障 */
	u8 bit_32_MCU_EEPROM_fault : 1;              /* 控制器 EEPROM 故障 */
	u8 bit_33_Motor_shortcircuit_fault : 1;      /* 电机绕组短路故障 */
	u8 bit_34_Motor_opencircuit_fault : 1;       /* 电机绕组开路故障 */
	u8 bit_35_Active_discharge_fault : 1;        /* 主动放电故障 */
	u8 : 0;
	
	u8 Live_Counter;                      /* 循环计数器 */
	
}MCU_STATUS4_t;

/*
数据解码计算方法：
物理量 =（数字量 * Scale）+ Offset
数字量 =（物理量 – Offset）/ Scale
*/


/******************************************************************************
														 发送数据处理函数声明
*******************************************************************************/
u8 VCU_COMMAND_SendData_Process(VCU_COMMAND_t vcu_cmd, u8 *sendbuf);
u8 VCU_COMMAND2_SendData_Process(VCU_COMMAND2_t vcu_cmd, u8 *sendbuf);
u8 VCU_COMMAD_ParaCheck(VCU_COMMAND_t vcu_cmd);
u8 VCU_COMMAD2_ParaCheck(VCU_COMMAND2_t vcu_cmd);


/******************************************************************************
														 接收数据解析函数声明
*******************************************************************************/
u8 MCU_STATUS1_ParseData(MCU_STATUS1_t *mcu_sta, u8 *recivebuf);
u8 MCU_STATUS2_ParseData(MCU_STATUS2_t *mcu_sta, u8 *recivebuf);
u8 MCU_STATUS3_ParseData(MCU_STATUS3_t *mcu_sta, u8 *recivebuf);
u8 MCU_STATUS4_ParseData(MCU_STATUS4_t *mcu_sta, u8 *recivebuf);





#endif



