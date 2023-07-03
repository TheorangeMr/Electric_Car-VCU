/*******************************************
	*	@file ：  main.c
	* @author：  罗成
	* @data： 2023.01.28
	* @version：  v1.0
*******************************************/


#include "stm32f10x.h"                  // Device header
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "queue.h"
#include "semphr.h"

#include "bsp_delay.h"
#include "adc.h"
#include "can.h"
#include "usart.h"
#include "HGW_Protocol.h"
#include "wit_c_sdk.h"


/*
*************************************************************************
*                            串口打印调试信息 
*************************************************************************
*/

#define USRAT_Memory_Throttle_Signal    0
#define USART_Throttle_Signal_1         0
#define MCU_STATUS1_Signal              0
#define MCU_STATUS2_Signal              0
#define MCU_STATUS3_Signal              0
#define MCU_STATUS4_Signal              0
#define VCU_Brake_Signal                0

/*
*************************************************************************
*                            内核对象句柄 
*************************************************************************
*/

EventGroupHandle_t EventGroupHandler = NULL;	
SemaphoreHandle_t  BinarySem1_Handle = NULL;
SemaphoreHandle_t  BinarySem2_Handle = NULL;

#define EVENTBIT_0	(1<<0)				//发送指令事件位
#define EVENTBIT_1	(1<<1)				//MCU_STATUS1指令事件位
#define EVENTBIT_2	(1<<2)				//MCU_STATUS2指令事件位
#define EVENTBIT_3	(1<<3)				//MCU_STATUS3指令事件位
#define EVENTBIT_4	(1<<4)				//MCU_STATUS4指令事件位
#define EVENTBIT_5	(1<<5)				//DBG_STATUS指令事件位
#define EVENTBIT_6	(1<<6)				//Wit_STATUS指令事件位

/*
*************************************************************************
*                             线程优先级
*************************************************************************
*/


#define   VCU_COMMAND_Pr             14
#define   VCU_BRAKE_Pr               15
#define   MCU_STATUS1_Pr							11
#define   MCU_STATUS2_Pr							10
#define   MCU_STATUS3_Pr							9
#define   MCU_STATUS4_Pr							8
#define   CAN_RX_HANDLE_Pr           16 
#define   Wit_Dat_HANDLE_Pr					 18

/*
*************************************************************************
*                             线程延时分配
*************************************************************************
*/


#define   VCU_COMMAND_Delay             7                      //VCU_COMMAND的发送周期为10sm       
#define   VCU_BRAKE_Delay               VCU_COMMAND_Delay*2    
#define   MCU_STATUS1_Delay							13                    //MCU STATUS1     10sm
#define   MCU_STATUS2_Delay							399                    //MCU STATUS2     20sm
#define   MCU_STATUS3_Delay							299                    //MCU STATUS3     20sm
#define   MCU_STATUS4_Delay							199                    //MCU STATUS4     50sm
#define   CAN_RX_HANDLE_Delay           1
#define   Wit_Dat_HANDLE_Delay          15

/*
*************************************************************************
*                             静态任务堆栈
*************************************************************************
*/

#define   App_Stack              256
#define   VCU_COMMAND_Stack      128

/*
*************************************************************************
*                             动态任务堆栈
*************************************************************************
*/

#define   MCU_STATUS1_Stack      128
#define   MCU_STATUS2_Stack      64
#define   MCU_STATUS3_Stack      64
#define   MCU_STATUS4_Stack      64
#define   VCU_Brake_Stack        128
#define   CAN_RX_HANDLE_Stack    256
#define   Wit_Dat_HANDLE_Stack   128


/*
*************************************************************************
*                             任务句柄初始化
*************************************************************************
*/

static TaskHandle_t AppTaskCreate_Handle = NULL;
static TaskHandle_t VCU_Brake_Task_Handle = NULL;
static TaskHandle_t VCU_COMMAND_Task_Handle = NULL;
static TaskHandle_t MCU_STATUS1_Task_Handle = NULL;
static TaskHandle_t MCU_STATUS2_Task_Handle = NULL;
static TaskHandle_t MCU_STATUS3_Task_Handle = NULL;
static TaskHandle_t MCU_STATUS4_Task_Handle = NULL;
static TaskHandle_t CAN_RX_HANDLE_Task_Handle = NULL;
static TaskHandle_t Wit_Dat_HANDLE_Task_Handle = NULL;

/*
*************************************************************************
*                             任务堆栈声明
*************************************************************************
*/


/* 闲置任务任务堆栈 */
static StackType_t Idle_Task_Stack[configMINIMAL_STACK_SIZE];
/* 定时器任务任务堆栈 */
static StackType_t Timer_Task_Stack[configTIMER_TASK_STACK_DEPTH];
/* 定义创建任务栈 */
static StackType_t AppTaskCreate_Stack[App_Stack];

/* VCU_COMMAND任务堆栈 */
static StackType_t VCU_COMMAND_Task_Stack[VCU_COMMAND_Stack];


/*
*************************************************************************
*                             控制块声明
*************************************************************************
*/

/* 闲置任务控制块 */
static StaticTask_t Idle_Task_TCB;
/* 定时器任务控制块 */
static StaticTask_t Timer_Task_TCB;
/* 定义创建任务控制块 */
static StaticTask_t AppTaskCreate_TCB;

static StaticTask_t VCU_COMMAND_Task_TCB;
/*
*************************************************************************
*                             函数声明 
*************************************************************************
*/


static void AppTaskCreate(void);                                                /* 用于创建任务 */
static void BSP_Init(void);
static void VCU_COMMAND_Send(void* parameter);
static void VCU_Brake_Task(void* parameter);
static void CAN_RX_HANDLE(void* parameter);
static void MCU_STATUS1_Show(void* parameter);
static void MCU_STATUS2_Show(void* parameter);
static void MCU_STATUS3_Show(void* parameter);
static void MCU_STATUS4_Show(void* parameter);
static void Wit_Dat_HANDLE(void* parameter);
static __INLINE void LINEAR_SPEED_DEAL(void);
static __INLINE void ADC_Get_Throttle_Signal(void);



/*
*************************************************************************
*                             全局变量
*************************************************************************
*/

#define Torque_Limit_High       500        //转矩上限
#define Torque_Limit_Low        20          //转矩下限
#define  Motorspeed             2000      //电机速度设置xxx（r/min）
#define  Sensitivity            100        //速度响应灵敏度
#define  Disable_Speed          50        //最低转速


CanTxMsg TxMessage;               //CAN发送信息结构体
CanRxMsg RxMessage;               //CAN接收信息结构体
VCU_COMMAND_t vcu_cmd;            // VCU_COMMAND命令参数结构体
int Throttle_Signal = 0;
int Memory_Throttle_Signal = 0;
uint8_t brake_flag = 0;          //制动标志
uint16_t correct = 0;
uint8_t Disable_Speed_flag = 0;   //低速标志


       
extern RegUpdateCb p_WitRegUpdateCbFunc;     //九轴数据 
extern volatile char  s_cDataUpdate;


u8 VCU_Command_Send[8];           //VCU_Command发送报文数组
u8 Status1_Buf[8] = {0};				 	/* MCU_STATUS1接收缓存数组 */
u8 Status2_Buf[8] = {0};				 	/* MCU_STATUS2接收缓存数组 */
u8 Status3_Buf[8] = {0};				 	/* MCU_STATUS3接收缓存数组 */
u8 Status4_Buf[8] = {0};				 	/* MCU_STATUS4接收缓存数组 */

vu8 MCU_Status1_Receive[8];       //MCU_STATUS1接收报文数组
vu8 MCU_Status2_Receive[8];       //MCU_STATUS2接收报文数组
vu8 MCU_Status3_Receive[8];       //MCU_STATUS3接收报文数组
vu8 MCU_Status4_Receive[8];       //MCU_STATUS4接收报文数组
vu8 DBG_Status_Receive[8];        //DBG_Status接收报文数组
MCU_STATUS1_t mcu_sta1;           /* MCU_STATUS1命令参数结构体 */
MCU_STATUS2_t mcu_sta2;           /* MCU_STATUS2命令参数结构体 */
MCU_STATUS3_t mcu_sta3;           /* MCU_STATUS3命令参数结构体 */
MCU_STATUS4_t mcu_sta4;           /* MCU_STATUS4命令参数结构体 */

/****************************************************************************/

/* 获取空闲任务的内存 */

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
	                                 StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize)
{
	*ppxIdleTaskTCBBuffer = &Idle_Task_TCB;
	*ppxIdleTaskStackBuffer = Idle_Task_Stack;
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/* 获取定时器任务的内存 */

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
	                                 StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize)
{
	*ppxTimerTaskTCBBuffer = &Timer_Task_TCB;
	*ppxTimerTaskStackBuffer = Timer_Task_Stack;
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}


/*****************************************************************
  * @brief  主函数
  * @param  无
  * @retval 无
  * @note   第一步：开发板硬件初始化 
            第二步：创建APP应用任务
            第三步：启动FreeRTOS，开始多任务调度
  ****************************************************************/
int main(void)
{	
	BSP_Init();
  printf("飞翼车队电车VCU函数调试!\r\n");
	AppTaskCreate_Handle = xTaskCreateStatic((TaskFunction_t	)AppTaskCreate,		  //任务函数
															(const char* 	)"AppTaskCreate",		                //任务名称
															(uint32_t 		)App_Stack,	                        //任务堆栈大小
															(void* 		  	)NULL,				                      //传递给任务函数的参数
															(UBaseType_t 	)3, 	                              //任务优先级
															(StackType_t*   )AppTaskCreate_Stack,	            //任务堆栈
															(StaticTask_t*  )&AppTaskCreate_TCB);	            //任务控制块 
  /* 启动任务调度 */          
	if(NULL != AppTaskCreate_Handle)                                              /* 创建成功 */
    vTaskStartScheduler();    
	else
		printf("创建失败! \r\n");
  
  while(1);   /* 正常不会执行到这里 */    
}

/**********************************************************************
  * @ API  ： BSP_Task
  * @ brief： BSP_Task任务主体
  * @ param    ：None
  * @ retval  ： None
  ********************************************************************/

void BSP_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Delay_Init(72);
	uart_init(115200);	//串口初始化为115200
	CAN_Config();       //CAN初始化
  Adc1_Init();	
  Adc2_Init();
	
	WitInit(WIT_PROTOCOL_CAN, 0x50);
	WitRegisterCallBack(SensorDataUpdata);
	WitCanWriteRegister(Wit_Can_Send_Msg);
	WitDelayMsRegister(Wit_Delayms);
	
}

/***********************************************************************
  * @ API  ： AppTaskCreate
  * @ brief： 为了方便管理，所有的任务创建函数都放在这个函数里面
  * @ param   ： None  
  * @ retval  ： None
  **********************************************************************/
static void AppTaskCreate(void)
{
	BaseType_t xReturn = pdPASS;     /* 定义一个创建信息返回值，默认为pdPASS */	
  taskENTER_CRITICAL();           //进入临界区

	/* 创建EventGroup */  
	EventGroupHandler = xEventGroupCreate();
	if(NULL != EventGroupHandler)
	printf("EventGroupHandler 事件创建成功！\r\n");	
	
	BinarySem1_Handle = xSemaphoreCreateBinary();
	BinarySem2_Handle = xSemaphoreCreateBinary();
	if(NULL != BinarySem1_Handle){
		printf("BinarySem1_Handle 二值信号量创建成功！\r\n");
	}	
	if(NULL != BinarySem2_Handle){
		CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);
		printf("BinarySem2_Handle 二值信号量创建成功！\r\n");
	}	
	
	  /* 创建VCU_BRAKE_TASK任务 */
	xReturn = xTaskCreate((TaskFunction_t	)VCU_Brake_Task,		               //任务函数
															(const char* 	)"VCU_Brake_Task",		         //任务名称
															(uint16_t 		)VCU_Brake_Stack,					     //任务堆栈大小
															(void* 		  	)NULL,  				               //传递给任务函数的参数
															(UBaseType_t 	)VCU_BRAKE_Pr, 				         //任务优先级
															(TaskHandle_t*  )&VCU_Brake_Task_Handle);	   //任务控制块指针   
	
	if(pdPASS == xReturn)/* 创建成功 */
		printf("VCU_BRAKE_TASK任务创建成功!\r\n");
	else
		printf("VCU_BRAKE_TASK任务创建失败!\r\n");

	  /* 创建VCU_COMMAND_SEND任务 */
	VCU_COMMAND_Task_Handle = xTaskCreateStatic((TaskFunction_t	)VCU_COMMAND_Send,//任务函数
															(const char* 	)"VCU_COMMAND_Send",		            //任务名称
															(uint32_t 		)VCU_COMMAND_Stack,					                      //任务堆栈大小
															(void* 		  	)NULL,				                      //传递给任务函数的参数
															(UBaseType_t 	)VCU_COMMAND_Pr, 				                        //任务优先级
															(StackType_t*   )VCU_COMMAND_Task_Stack,	        //任务堆栈
															(StaticTask_t*  )&VCU_COMMAND_Task_TCB);	        //任务控制块  
	
	if(NULL != VCU_COMMAND_Task_Handle)                                                   /* 创建成功 */
		printf("VCU_COMMAND_Send任务创建成功!\r\n");
	else
		printf("VCU_COMMAND_Send任务创建失败!\r\n");
	
	  /* 创建MCU_STATUS1_Show任务 */
	xReturn = xTaskCreate((TaskFunction_t	)MCU_STATUS1_Show,		               //任务函数
															(const char* 	)"MCU_STATUS1_Show",		         //任务名称
															(uint16_t 		)MCU_STATUS1_Stack,					     //任务堆栈大小
															(void* 		  	)NULL,  				               //传递给任务函数的参数
															(UBaseType_t 	)MCU_STATUS1_Pr, 				         //任务优先级
															(TaskHandle_t*  )&MCU_STATUS1_Task_Handle);	   //任务控制块指针   
	
	if(pdPASS == xReturn)/* 创建成功 */
		printf("MCU_STATUS1_Show任务创建成功!\r\n");
	else
		printf("MCU_STATUS1_Show任务创建失败!\r\n");
	
	  /* 创建MCU_STATUS2_Show任务 */
	xReturn = xTaskCreate((TaskFunction_t	)MCU_STATUS2_Show,		               //任务函数
															(const char* 	)"MCU_STATUS2_Show",		         //任务名称
															(uint16_t 		)MCU_STATUS2_Stack,					     //任务堆栈大小
															(void* 		  	)NULL,  				               //传递给任务函数的参数
															(UBaseType_t 	)MCU_STATUS2_Pr, 				         //任务优先级
															(TaskHandle_t*  )&MCU_STATUS2_Task_Handle);	   //任务控制块指针   
	
	if(pdPASS == xReturn)/* 创建成功 */
		printf("MCU_STATUS2_Show任务创建成功!\r\n");
	else
		printf("MCU_STATUS2_Show任务创建失败!\r\n");
	
	
	  /* 创建MCU_STATUS3_Show任务 */
	xReturn = xTaskCreate((TaskFunction_t	)MCU_STATUS3_Show,		               //任务函数
															(const char* 	)"MCU_STATUS3_Show",		         //任务名称
															(uint16_t 		)MCU_STATUS3_Stack,					     //任务堆栈大小
															(void* 		  	)NULL,  				               //传递给任务函数的参数
															(UBaseType_t 	)MCU_STATUS3_Pr, 				         //任务优先级
															(TaskHandle_t*  )&MCU_STATUS3_Task_Handle);	   //任务控制块指针   
	
	if(pdPASS == xReturn)/* 创建成功 */
		printf("MCU_STATUS3_Show任务创建成功!\r\n");
	else
		printf("MCU_STATUS3_Show任务创建失败!\r\n");	
	
	  /* 创建MCU_STATUS4_Show任务 */
	xReturn = xTaskCreate((TaskFunction_t	)MCU_STATUS4_Show,		               //任务函数
															(const char* 	)"MCU_STATUS4_Show",		         //任务名称
															(uint16_t 		)MCU_STATUS4_Stack,					     //任务堆栈大小
															(void* 		  	)NULL,  				               //传递给任务函数的参数
															(UBaseType_t 	)MCU_STATUS4_Pr, 				         //任务优先级
															(TaskHandle_t*  )&MCU_STATUS4_Task_Handle);	   //任务控制块指针   
	
	if(pdPASS == xReturn)/* 创建成功 */
		printf("MCU_STATUS4_Show任务创建成功!\r\n");
	else
		printf("MCU_STATUS4_Show任务创建失败!\r\n");	
	
		  /* 创建CAN_RX_HANDLE任务 */
	xReturn = xTaskCreate((TaskFunction_t	)CAN_RX_HANDLE,		                 //任务函数
															(const char* 	)"CAN_RX_HANDLE",		           //任务名称
															(uint16_t 		)CAN_RX_HANDLE_Stack,					 //任务堆栈大小
															(void* 		  	)NULL,  				               //传递给任务函数的参数
															(UBaseType_t 	)CAN_RX_HANDLE_Pr, 				     //任务优先级
															(TaskHandle_t*  )&CAN_RX_HANDLE_Task_Handle);	   //任务控制块指针   
	
	if(pdPASS == xReturn)/* 创建成功 */
		printf("CAN_RX_HANDLE任务创建成功!\r\n");
	else
		printf("CAN_RX_HANDLE任务创建失败!\r\n");	
	
		  /* 创建Wit_Dat_HANDLE任务 */
	xReturn = xTaskCreate((TaskFunction_t	)Wit_Dat_HANDLE,		                 //任务函数
															(const char* 	)"Wit_Dat_HANDLE",		           //任务名称
															(uint16_t 		)Wit_Dat_HANDLE_Stack,					 //任务堆栈大小
															(void* 		  	)NULL,  				               //传递给任务函数的参数
															(UBaseType_t 	)Wit_Dat_HANDLE_Pr, 				     //任务优先级
															(TaskHandle_t*  )&Wit_Dat_HANDLE_Task_Handle);	   //任务控制块指针   
	
	if(pdPASS == xReturn)/* 创建成功 */
		printf("Wit_Dat_HANDLE任务创建成功!\r\n");
	else
		printf("Wit_Dat_HANDLE任务创建失败!\r\n");		
	

	
  vTaskDelete(AppTaskCreate_Handle);                                            //删除AppTaskCreate任务
    
  taskEXIT_CRITICAL();                                                          //退出临界区
}




/**********************************************************************
  * @ API  ： VCU_COMMAND_Send
  * @ brief： VCU_COMMAND_Send任务主体
  * @ param    ：None   
  * @ retval  ： None
  ********************************************************************/

static void VCU_COMMAND_Send(void* parameter) //VCU_COMMAND配置
{
	while(1)
	{
		xSemaphoreTake(BinarySem1_Handle,portMAX_DELAY);
			/* 初始化命令参数 */
		vcu_cmd.MCU_Enable = 1;
		vcu_cmd.Fault_Reset = 0;
		vcu_cmd.Control_Mode = Speed_Control;
		vcu_cmd.Demand_Limit_High = Torque_Limit_High;    //转矩上限
		vcu_cmd.Demand_Limit_Low = Torque_Limit_Low;     //转速下限
		vcu_cmd.Live_Counter = 0;
		vcu_cmd.Demand_Torque = 0;
		ADC_Get_Throttle_Signal();
		#if USRAT_Memory_Throttle_Signal
		printf("Memory_Throttle_Signal = %d\r\n",Memory_Throttle_Signal);
		#endif
		#if USART_Throttle_Signal_1
		printf("Throttle_Signal = %d\r\n",Throttle_Signal);
		#endif
		LINEAR_SPEED_DEAL();					
		vTaskDelay(VCU_COMMAND_Delay);
	}
}

/**********************************************************************
  * @ 函数名  ： MCU_STATUS1_Show
  * @ 功能说明： 接收MCU_STATUS1指令函数
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/

static void MCU_STATUS1_Show(void* parameter)
{
  u8 res = 0,i=0;
	EventBits_t r_event;
	while(1)
	{
		r_event = xEventGroupWaitBits(EventGroupHandler,EVENTBIT_1,pdTRUE,pdTRUE,portMAX_DELAY);
		if((r_event&EVENTBIT_1) == EVENTBIT_1)
		{
			for(i = 0; i < 8; i++)
			{
				Status1_Buf[i] = MCU_Status1_Receive[i];
			}
			/* MCU_STATUS1命令接收数据解析 */
			res = MCU_STATUS1_ParseData(&mcu_sta1, Status1_Buf);
			if(res == 0)/* 加工完成 */
			{
				#if MCU_STATUS1_Signal
				printf("STATUS1 OK!\r\n");
				printf("Motor_Speed = %d",mcu_sta1.Motor_Speed);
				printf("Motor_Phase_Current = %d",mcu_sta1.Motor_Phase_Current);
				printf("Precharge_Allow = %d",mcu_sta1.Precharge_Allow);
				printf("MCU_Enable_Feedback = %d",mcu_sta1.MCU_Enable_Feedback);
				printf("Work_Mode = %d",mcu_sta1.Work_Mode);	
				printf("Live_Counter = %d",mcu_sta1.Live_Counter);
				#endif
			 }
			 else if(res == 1)
			 printf("STATUS1 NO!\r\n");
		 }
		vTaskDelay(MCU_STATUS1_Delay);
	}
}


/**********************************************************************
  * @ API  ： MCU_STATUS2_Show
  * @ brief： 接收MCU_STATUS2指令函数
  * @ param    ： None  
  * @ retval  ： None
  ********************************************************************/


static void MCU_STATUS2_Show(void* parameter)
{
  u8 i=0,res = 0;
	EventBits_t r_event;
	while(1)
	{
		r_event = xEventGroupWaitBits(EventGroupHandler,EVENTBIT_2,pdTRUE,pdTRUE,portMAX_DELAY);
		if((r_event&EVENTBIT_2) == EVENTBIT_2)
		{
			for(i = 0; i < 8; i++)
			{
				Status2_Buf[i] = MCU_Status2_Receive[i];
			}
			/* MCU_STATUS1命令接收数据解析 */
			res = MCU_STATUS2_ParseData(&mcu_sta2, Status2_Buf);
			if(res == 0)/* 加工完成 */
			{
				#if MCU_STATUS2_Signal
				printf("STATUS2 OK!\r\n");
				#endif
			}
			 else if(res == 1)
			 printf("STATUS2 NO!\r\n");
		}
		vTaskDelay(MCU_STATUS2_Delay);
	}
}


/**********************************************************************
  * @ API  ： MCU_STATUS3_Show
  * @ brief： 接收MCU_STATUS4指令函数
  * @ param   ： None
  * @ retval  ： None
  ********************************************************************/


static void MCU_STATUS3_Show(void* parameter)
{
  u8 i=0,res = 0;
	EventBits_t r_event;
	while(1)
	{
		r_event = xEventGroupWaitBits(EventGroupHandler,EVENTBIT_3,pdTRUE,pdTRUE,portMAX_DELAY);
		if((r_event&EVENTBIT_3) == EVENTBIT_3)
		{
			for(i = 0; i < 8; i++)
			{
				Status3_Buf[i] = MCU_Status3_Receive[i];
			}
			/* MCU_STATUS1命令接收数据解析 */
			res = MCU_STATUS3_ParseData(&mcu_sta3, Status3_Buf);
			if(res == 0)/* 加工完成 */
			{
				#if MCU_STATUS3_Signal
				printf("STATUS3 OK!\r\n");
				#endif
			}
			 else if(res == 1)
			 printf("STATUS3 NO!\r\n");
		 }
		vTaskDelay(MCU_STATUS3_Delay);
	}
}

/**********************************************************************
  * @ API  ： MCU_STATUS4_Show
  * @ brief： 接收MCU_STATUS4指令函数
  * @ param   ：   
  * @ retval  ： None
  ********************************************************************/

static void MCU_STATUS4_Show(void* parameter)
{
  u8 i=0,res = 0;
	EventBits_t r_event;
	while(1)
	{
		r_event = xEventGroupWaitBits(EventGroupHandler,EVENTBIT_4,pdTRUE,pdTRUE,portMAX_DELAY);
		if((r_event&EVENTBIT_4) == EVENTBIT_4)
		{
			for(i = 0; i < 8; i++)
			{
				Status4_Buf[i] = MCU_Status4_Receive[i];
			}
			/* MCU_STATUS1命令接收数据解析 */
			res = MCU_STATUS4_ParseData(&mcu_sta4, Status4_Buf);
			if(res == 0)/* 加工完成 */
			{
				#if MCU_STATUS4_Signal
				printf("STATUS4 OK!\r\n");
				#endif
			}
			 else if(res == 1)
			 printf("STATUS4 NO!\r\n");
		 }
		 vTaskDelay(MCU_STATUS4_Delay);
	}
}

/**********************************************************************
  * @ API  ： VCU_Brake_Task
  * @ brief： 判断制动踏板液压传感器的值
  * @ param： None
  * @ retval：None
  ********************************************************************/

static void VCU_Brake_Task(void* parameter)
{
	uint8_t res = 0;
	static uint8_t b = 0;
	float adc2_dat = 0,PCM300D_dat = 0;
	while(1)
	{
		adc2_dat = ADC_SingleMode_GetAverageValue(ADC2,ADC_Channel_2);
		#if VCU_Brake_Signal
		printf("VCU_BRAKE_Delay  \r\n");
		printf(" CH2 value = %f mV \r\n",adc2_dat);
		#endif
		PCM300D_dat = adc2_dat*3.3/4096;//PCM300D_Constant_Pa
		while(PCM300D_dat >= 2)
		{
			brake_flag = 1;
			b = 0;
			vcu_cmd.MCU_Enable = 0; //电机控制器不使能
			res = VCU_COMMAND_SendData_Process(vcu_cmd, VCU_Command_Send);
			if(res == 0) /* 加工完成 */
			{
				res = Can_Send_Msg(VCU_Command, VCU_Command_Send, 8);//发送8个字节
			}
		  vTaskDelay(10);
			adc2_dat = ADC_SingleMode_GetAverageValue(ADC2,ADC_Channel_2);
			PCM300D_dat = adc2_dat*3.3/4096;
		}
		if(brake_flag == 1)
		{
			if(b == 0)
			{
				if((ADC_SingleMode_GetAverageValue(ADC1,ADC_Channel_1)-correct)*Motorspeed*1.0/3300 < Disable_Speed*2)
				{
					Memory_Throttle_Signal = 0;
					b = 1;
				}				
			}
			if(b == 1)
			{
				if((ADC_SingleMode_GetAverageValue(ADC1,ADC_Channel_1)-correct)*Motorspeed*1.0/3300 > mcu_sta1.Motor_Speed)
				{
					b = 0;
					brake_flag = 0;
					xSemaphoreGive(BinarySem1_Handle);
				}				
			}
		}
		else
		{
			xSemaphoreGive(BinarySem1_Handle);			
		}
		 vTaskDelay(VCU_BRAKE_Delay);
	}
}

/**********************************************************************
  * @ API  ： CAN_RX_HANDLE
  * @ brief： 处理CAN接受的报文
  * @ param： None 
  * @ retval：None
  ********************************************************************/
static void CAN_RX_HANDLE(void* parameter)
{
	uint8_t i = 0;
	while(1)
	{
  	xSemaphoreTake(BinarySem2_Handle,portMAX_DELAY);
				/* 初始化CAN接收器 */
		RxMessage.StdId=0x00;
		RxMessage.ExtId=0x00;
		RxMessage.IDE=0;
		RxMessage.DLC=0;
		RxMessage.FMI=0;
		for(i = 0; i < 8; i++)
		{
			RxMessage.Data[i]=0x00;
		}
		/* 接收数据 */
		CAN_Receive(CANx, CAN_FIFO0, &RxMessage);
		/* 判断是否为目标方发送,并对数据进行校验 */	
		if((RxMessage.ExtId==MCU_Status1) && (RxMessage.IDE==CAN_Id_Extended) && (RxMessage.DLC==8))
		{
			/*事件置位*/
//			printf("MCU_Status1\r\n");
			for(i = 0; i < 8; i++) {MCU_Status1_Receive[i] = RxMessage.Data[i];}
			xEventGroupSetBits(EventGroupHandler,EVENTBIT_1);
		}
		else if((RxMessage.ExtId==MCU_Status2) && (RxMessage.IDE==CAN_Id_Extended) && (RxMessage.DLC==8))
		{
			for(i = 0; i < 8; i++) {MCU_Status2_Receive[i] = RxMessage.Data[i];}
			xEventGroupSetBits(EventGroupHandler,EVENTBIT_2);
		}
		else if((RxMessage.ExtId==MCU_Status3) && (RxMessage.IDE==CAN_Id_Extended) && (RxMessage.DLC==8))
		{
			for(i = 0; i < 8; i++) {MCU_Status3_Receive[i] = RxMessage.Data[i];}
			xEventGroupSetBits(EventGroupHandler,EVENTBIT_3); 
		}
		 else if((RxMessage.ExtId==MCU_Status4) && (RxMessage.IDE==CAN_Id_Extended) && (RxMessage.DLC==8))
		{
			for(i = 0; i < 8; i++) {MCU_Status4_Receive[i] = RxMessage.Data[i];}
			xEventGroupSetBits(EventGroupHandler,EVENTBIT_4);
		}
		else if((RxMessage.ExtId==DBG_Status) && (RxMessage.IDE==CAN_Id_Extended) && (RxMessage.DLC==8))
		{
			for(i = 0; i < 8; i++) {DBG_Status_Receive[i] = RxMessage.Data[i];}
			xEventGroupSetBits(EventGroupHandler,EVENTBIT_5);
		}
		//九轴CAN报文
		if((RxMessage.StdId==Wit_dat) && (RxMessage.IDE==CAN_Id_Standard) && (RxMessage.DLC==8))
		{
			WitCanDataIn(RxMessage.Data, RxMessage.DLC);
			xEventGroupSetBits(EventGroupHandler,EVENTBIT_6);
		}
	}
}



/**********************************************************************
  * @ API  ： LINEAR_SPEED_DEAL
  * @ brief： 处理adc数据，判断加速，减速
  * @ param： None 
  * @ retval：None
  ********************************************************************/
static __INLINE void LINEAR_SPEED_DEAL(void)
{
	 uint8_t res = 0;
	 int difference_value = 0;
	 uint8_t Motor_status = 0;
	 difference_value = Memory_Throttle_Signal - Throttle_Signal;
	 

	 if(difference_value > Sensitivity){Motor_status = 1;}                   //减速
	 else if(difference_value + Sensitivity >= 0&&difference_value <= Sensitivity){Motor_status = 2;}//匀速
	 else if(difference_value + Sensitivity< 0){Motor_status = 3;}           //加速
//	 printf("Motor_status = %d\r\n",Motor_status);	 
	 switch(Motor_status)
	 {
		 case(1):
			{
				if(Disable_Speed_flag == 1)
				{
					Disable_Speed_flag:
					if(Throttle_Signal>=Disable_Speed)
					{
						if(mcu_sta1.Motor_Speed > Throttle_Signal)
						{
							vcu_cmd.MCU_Enable = 0;
							res = VCU_COMMAND_SendData_Process(vcu_cmd, VCU_Command_Send);
							if(res == 0) /* 加工完成 */
							{
								res = Can_Send_Msg(VCU_Command, VCU_Command_Send, 8);//发送8个字节
							}
							vTaskDelay(15);
							goto Disable_Speed_flag;
						}
						vcu_cmd.Demand_Speed = Throttle_Signal;
						vTaskDelay(50);                                                //调参值
					}
					Disable_Speed_flag = 0;
				}
				else
				{
	        if(Throttle_Signal>=Disable_Speed)
					{
						vcu_cmd.Demand_Speed = Throttle_Signal;				
					}
					else
					{
						//掉电，电机变为空挡
						Memory_Throttle_Signal = 0;
						vcu_cmd.MCU_Enable = 0;                                          //电机控制器不使能
					}
				}
				res = VCU_COMMAND_SendData_Process(vcu_cmd, VCU_Command_Send);
				if(res == 0) /* 加工完成 */
				{
					res = Can_Send_Msg(VCU_Command, VCU_Command_Send, 8);//发送8个字节
				}
			  break;
			}
		 case(2):
			{
				if(Memory_Throttle_Signal > Throttle_Signal)
				{
					 if(Memory_Throttle_Signal >= Disable_Speed)
					 {
							vcu_cmd.Demand_Speed = Memory_Throttle_Signal;
					 }
					 else
					 {	//掉电，电机变为空挡
						 	Memory_Throttle_Signal = 0;
							vcu_cmd.MCU_Enable = 0;       //电机控制器不使能
					 }
					 res = VCU_COMMAND_SendData_Process(vcu_cmd, VCU_Command_Send);
					 if(res == 0) /* 加工完成 */
					 {
							res = Can_Send_Msg(VCU_Command, VCU_Command_Send, 8);//发送8个字节
					 }
				}
				else
				{
					 if(Throttle_Signal >= Disable_Speed)
					 {
							vcu_cmd.Demand_Speed = Throttle_Signal;
					 }
					 else
					 {
						 Memory_Throttle_Signal = 0;
						 vcu_cmd.MCU_Enable = 0;       //电机控制器不使能
					 }
					 res = VCU_COMMAND_SendData_Process(vcu_cmd, VCU_Command_Send);
					 if(res == 0) /* 加工完成 */
					 {
							res = Can_Send_Msg(VCU_Command, VCU_Command_Send, 8);//发送8个字节
					 }
				}
				break;
			}
		 case(3):
			{
				if(Throttle_Signal >= Disable_Speed)
				{
					if(Throttle_Signal < mcu_sta1.Motor_Speed)
					{
						vcu_cmd.Demand_Speed = mcu_sta1.Motor_Speed;
					}
					else
					{
						vcu_cmd.Demand_Speed = Throttle_Signal;					 //Memory_Throttle_Signal
					}
				}
				else
				{
					 Disable_Speed_flag = 1;
					 Memory_Throttle_Signal = 0;
					 vcu_cmd.MCU_Enable = 0;       //电机控制器不使能
				}
				res = VCU_COMMAND_SendData_Process(vcu_cmd, VCU_Command_Send);
				if(res == 0) /* 加工完成 */
				{
						res = Can_Send_Msg(VCU_Command, VCU_Command_Send, 8);//发送8个字节
				}
				break;
			}
		 default:
		break;
	 }
 }


/**********************************************************************
  * @ API  ： ADC_Get_Throttle_Signal
  * @ brief： 检测油门量程
  * @ param   ： None 
  * @ retval  ： None
  ********************************************************************/

static __INLINE void ADC_Get_Throttle_Signal(void) //ADC获取油门信号
{
	static u16 i,a;
	float adc = 0;
	if(a == 0)
	{
		if(i == 5)
		{
			for(i=5;i<55;i++)
			{
				correct += ADC_SingleMode_GetAverageValue(ADC1,ADC_Channel_1);
			}
			correct =correct*0.02;           //0.02 = (1/50)
			a = 1;
		}
		i++;
	}
	else
	{
		Memory_Throttle_Signal = Throttle_Signal;
		adc = ADC_SingleMode_GetAverageValue(ADC1,ADC_Channel_1)-correct;
		Throttle_Signal = adc*(Motorspeed*1.0/3300);
	}
}



/**********************************************************************
  * @ API  ： Wit_Dat_HANDLE
  * @ brief： 处理九轴传感接受的报文
  * @ param： None 
  * @ retval：None
  ********************************************************************/
static void Wit_Dat_HANDLE(void* parameter)
{
	int i;
	float fAcc[3], fGyro[3], fAngle[3];
	EventBits_t r_event;
	while(1)
	{
		r_event = xEventGroupWaitBits(EventGroupHandler,EVENTBIT_6,pdTRUE,pdTRUE,portMAX_DELAY);
		if((r_event&EVENTBIT_6) == EVENTBIT_6)
		{
			CmdProcess();
			if(s_cDataUpdate)
			{
				for(i = 0; i < 3; i++)
				{
					fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
					fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
					fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
				}
				if(s_cDataUpdate & ACC_UPDATE)
				{
					printf("acc:%.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);
					s_cDataUpdate &= ~ACC_UPDATE;
				}
				if(s_cDataUpdate & GYRO_UPDATE)
				{
					printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
					s_cDataUpdate &= ~GYRO_UPDATE;
				}
				if(s_cDataUpdate & ANGLE_UPDATE)
				{
					printf("angle:%.3f %.3f %.3f\r\n", fAngle[0], fAngle[1], fAngle[2]);
					s_cDataUpdate &= ~ANGLE_UPDATE;
				}
				if(s_cDataUpdate & MAG_UPDATE)
				{
					printf("mag:%d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]);
					s_cDataUpdate &= ~MAG_UPDATE;
				}
			}
			vTaskDelay(Wit_Dat_HANDLE_Delay);
		}
	}
}

