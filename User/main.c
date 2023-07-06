/*******************************************
	*	@file ��  main.c
	* @author��  �޳�
	* @data�� 2023.01.28
	* @version��  v1.0
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
#include "dangwei.h"

/*
*************************************************************************
*                            ���ڴ�ӡ������Ϣ 
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
*                            �ں˶����� 
*************************************************************************
*/

EventGroupHandle_t EventGroupHandler = NULL;	
SemaphoreHandle_t  BinarySem1_Handle = NULL;
SemaphoreHandle_t  BinarySem2_Handle = NULL;
SemaphoreHandle_t  BinarySem3_Handle = NULL;


//SemaphoreHandle_t  MuxSem_Handle = NULL;    //�����ź������

#define EVENTBIT_0	(1<<0)				//����ָ���¼�λ
#define EVENTBIT_1	(1<<1)				//MCU_STATUS1ָ���¼�λ
#define EVENTBIT_2	(1<<2)				//MCU_STATUS2ָ���¼�λ
#define EVENTBIT_3	(1<<3)				//MCU_STATUS3ָ���¼�λ
#define EVENTBIT_4	(1<<4)				//MCU_STATUS4ָ���¼�λ
#define EVENTBIT_5	(1<<5)				//DBG_STATUSָ���¼�λ
#define EVENTBIT_6	(1<<6)				//Wit_STATUSָ���¼�λ

/*
*************************************************************************
*                             �߳����ȼ�
*************************************************************************
*/


#define   VCU_COMMAND_Pr             14
#define   VCU_BRAKE_Pr               15
#define   MCU_STATUS1_Pr							11
#define   MCU_STATUS2_Pr							10
#define   MCU_STATUS3_Pr							9
#define   MCU_STATUS4_Pr							8
#define   CAN_RX_HANDLE_Pr           16 
#define   Wit_Dat_HANDLE_Pr					 6
#define   Dangwei_HANDLE_Pr					 17	
#define   Dangwei_Scan_Pr            7


/*
*************************************************************************
*                             �߳���ʱ����
*************************************************************************
*/


#define   VCU_COMMAND_Delay             7                      //VCU_COMMAND�ķ�������Ϊ10sm       
#define   VCU_BRAKE_Delay               VCU_COMMAND_Delay*2    
#define   MCU_STATUS1_Delay							13                    //MCU STATUS1     10sm
#define   MCU_STATUS2_Delay							399                    //MCU STATUS2     20sm
#define   MCU_STATUS3_Delay							299                    //MCU STATUS3     20sm
#define   MCU_STATUS4_Delay							199                    //MCU STATUS4     50sm
#define   CAN_RX_HANDLE_Delay           1
#define   Wit_Dat_HANDLE_Delay          15
#define   Dangwei_HANDLE_Delay          9
#define   Dangwei_Scan_Delay          	99



/*
*************************************************************************
*                             ��̬�����ջ
*************************************************************************
*/

#define   App_Stack              256
#define   VCU_COMMAND_Stack      128

/*
*************************************************************************
*                             ��̬�����ջ
*************************************************************************
*/

#define   MCU_STATUS1_Stack      128
#define   MCU_STATUS2_Stack      64
#define   MCU_STATUS3_Stack      64
#define   MCU_STATUS4_Stack      64
#define   VCU_Brake_Stack        128
#define   CAN_RX_HANDLE_Stack    256
#define   Wit_Dat_HANDLE_Stack   64
#define   Dangwei_HANDLE_Stack   128
#define   Dangwei_Scan_Stack   	 64


/*
*************************************************************************
*                             ��������ʼ��
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
static TaskHandle_t Dangwei_HANDLE_Task_Handle = NULL;
static TaskHandle_t Dangwei_Scan_Task_Handle = NULL;


/*
*************************************************************************
*                             �����ջ����
*************************************************************************
*/


/* �������������ջ */
static StackType_t Idle_Task_Stack[configMINIMAL_STACK_SIZE];
/* ��ʱ�����������ջ */
static StackType_t Timer_Task_Stack[configTIMER_TASK_STACK_DEPTH];
/* ���崴������ջ */
static StackType_t AppTaskCreate_Stack[App_Stack];

/* VCU_COMMAND�����ջ */
static StackType_t VCU_COMMAND_Task_Stack[VCU_COMMAND_Stack];


/*
*************************************************************************
*                             ���ƿ�����
*************************************************************************
*/

/* ����������ƿ� */
static StaticTask_t Idle_Task_TCB;
/* ��ʱ��������ƿ� */
static StaticTask_t Timer_Task_TCB;
/* ���崴��������ƿ� */
static StaticTask_t AppTaskCreate_TCB;

static StaticTask_t VCU_COMMAND_Task_TCB;
/*
*************************************************************************
*                             �������� 
*************************************************************************
*/


static void AppTaskCreate(void);                                                /* ���ڴ������� */
static void BSP_Init(void);
static void VCU_COMMAND_Send(void* parameter);
static void VCU_Brake_Task(void* parameter);
static void CAN_RX_HANDLE(void* parameter);
static void MCU_STATUS1_Show(void* parameter);
static void MCU_STATUS2_Show(void* parameter);
static void MCU_STATUS3_Show(void* parameter);
static void MCU_STATUS4_Show(void* parameter);
static void Wit_Dat_HANDLE(void* parameter);
static void Dangwei_HANDLE(void* parameter);
static void Dangwei_Scan(void* parameter);
static __INLINE void LINEAR_SPEED_DEAL(void);
static __INLINE void ADC_Get_Throttle_Signal(void);



/*
*************************************************************************
*                             ȫ�ֱ���
*************************************************************************
*/

#define D_Torque_Limit_High       500        //ת������
#define D_Torque_Limit_Low        20          //ת������
#define R_Torque_Limit_High       1000        //ת������
#define R_Torque_Limit_Low        20          //ת������

#define  Motorspeed             2000      //����ٶ�����xxx��r/min��
#define  Sensitivity            100        //�ٶ���Ӧ������
#define  Disable_Speed          50        //���ת��


CanTxMsg TxMessage;               //CAN������Ϣ�ṹ��
CanRxMsg RxMessage;               //CAN������Ϣ�ṹ��
VCU_COMMAND_t vcu_cmd;            // VCU_COMMAND��������ṹ��
VCU_COMMAND2_t vcu_cmd2;            // VCU_COMMAND2��������ṹ��
int Throttle_Signal = 0;
int Memory_Throttle_Signal = 0;
uint8_t brake_flag = 0;          //�ƶ���־
uint16_t correct = 0;
uint8_t Disable_Speed_flag = 0;   //���ٱ�־

uint8_t Gears_flag = 0;

uint8_t Gears_status = 0;       
       
extern RegUpdateCb p_WitRegUpdateCbFunc;     //�������� 
extern volatile char  s_cDataUpdate;


u8 VCU_Command_Send[8];           //VCU_Command���ͱ�������
u8 Status1_Buf[8] = {0};				 	/* MCU_STATUS1���ջ������� */
u8 Status2_Buf[8] = {0};				 	/* MCU_STATUS2���ջ������� */
u8 Status3_Buf[8] = {0};				 	/* MCU_STATUS3���ջ������� */
u8 Status4_Buf[8] = {0};				 	/* MCU_STATUS4���ջ������� */

vu8 MCU_Status1_Receive[8];       //MCU_STATUS1���ձ�������
vu8 MCU_Status2_Receive[8];       //MCU_STATUS2���ձ�������
vu8 MCU_Status3_Receive[8];       //MCU_STATUS3���ձ�������
vu8 MCU_Status4_Receive[8];       //MCU_STATUS4���ձ�������
vu8 DBG_Status_Receive[8];        //DBG_Status���ձ�������
MCU_STATUS1_t mcu_sta1;           /* MCU_STATUS1��������ṹ�� */
MCU_STATUS2_t mcu_sta2;           /* MCU_STATUS2��������ṹ�� */
MCU_STATUS3_t mcu_sta3;           /* MCU_STATUS3��������ṹ�� */
MCU_STATUS4_t mcu_sta4;           /* MCU_STATUS4��������ṹ�� */

/****************************************************************************/

/* ��ȡ����������ڴ� */

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
	                                 StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize)
{
	*ppxIdleTaskTCBBuffer = &Idle_Task_TCB;
	*ppxIdleTaskStackBuffer = Idle_Task_Stack;
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/* ��ȡ��ʱ��������ڴ� */

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
	                                 StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize)
{
	*ppxTimerTaskTCBBuffer = &Timer_Task_TCB;
	*ppxTimerTaskStackBuffer = Timer_Task_Stack;
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}


/*****************************************************************
  * @brief  ������
  * @param  ��
  * @retval ��
  * @note   ��һ����������Ӳ����ʼ�� 
            �ڶ���������APPӦ������
            ������������FreeRTOS����ʼ���������
  ****************************************************************/
int main(void)
{	
	BSP_Init();
  printf("�����ӵ糵VCU��������!\r\n");
	AppTaskCreate_Handle = xTaskCreateStatic((TaskFunction_t	)AppTaskCreate,		  //������
															(const char* 	)"AppTaskCreate",		                //��������
															(uint32_t 		)App_Stack,	                        //�����ջ��С
															(void* 		  	)NULL,				                      //���ݸ��������Ĳ���
															(UBaseType_t 	)3, 	                              //�������ȼ�
															(StackType_t*   )AppTaskCreate_Stack,	            //�����ջ
															(StaticTask_t*  )&AppTaskCreate_TCB);	            //������ƿ� 
  /* ����������� */          
	if(NULL != AppTaskCreate_Handle)                                              /* �����ɹ� */
    vTaskStartScheduler();    
	else
		printf("����ʧ��! \r\n");
  
  while(1);   /* ��������ִ�е����� */    
}

/**********************************************************************
  * @ API  �� BSP_Task
  * @ brief�� BSP_Task��������
  * @ param    ��None
  * @ retval  �� None
  ********************************************************************/

void BSP_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Delay_Init(72);
	uart_init(115200);	//���ڳ�ʼ��Ϊ115200
	CAN_Config();       //CAN��ʼ��
  Adc1_Init();
  Adc2_Init();
	dangwei_gpio_init();
	
	WitInit(WIT_PROTOCOL_CAN, 0x50);
	WitRegisterCallBack(SensorDataUpdata);
	WitCanWriteRegister(Wit_Can_Send_Msg);
	WitDelayMsRegister(Wit_Delayms);
}

/***********************************************************************
  * @ API  �� AppTaskCreate
  * @ brief�� Ϊ�˷���������е����񴴽����������������������
  * @ param   �� None  
  * @ retval  �� None
  **********************************************************************/
static void AppTaskCreate(void)
{
	BaseType_t xReturn = pdPASS;     /* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */	
  taskENTER_CRITICAL();           //�����ٽ���

	
	/* ����MuxSem */
//  MuxSem_Handle = xSemaphoreCreateMutex();	 
//  if(NULL != MuxSem_Handle)
//    printf("MuxSem_Handle�����������ɹ�!\r\n");

//  xReturn = xSemaphoreGive( MuxSem_Handle );//����������
//  if( xReturn == pdTRUE )
//  printf("�ͷ��ź���!\r\n");
	
	/* ����EventGroup */  
	EventGroupHandler = xEventGroupCreate();
	if(NULL != EventGroupHandler)
	printf("EventGroupHandler �¼������ɹ���\r\n");	
	
	BinarySem1_Handle = xSemaphoreCreateBinary();
	BinarySem2_Handle = xSemaphoreCreateBinary();
	BinarySem3_Handle = xSemaphoreCreateBinary();
	if(NULL != BinarySem1_Handle){
		printf("BinarySem1_Handle ��ֵ�ź��������ɹ���\r\n");
	}	
	if(NULL != BinarySem2_Handle){
		CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);
		printf("BinarySem2_Handle ��ֵ�ź��������ɹ���\r\n");
	}	
	if(NULL != BinarySem3_Handle){
		printf("BinarySem3_Handle ��ֵ�ź��������ɹ���\r\n");
	}		
	  /* ����VCU_BRAKE_TASK���� */
	xReturn = xTaskCreate((TaskFunction_t	)VCU_Brake_Task,		               //������
															(const char* 	)"VCU_Brake_Task",		         //��������
															(uint16_t 		)VCU_Brake_Stack,					     //�����ջ��С
															(void* 		  	)NULL,  				               //���ݸ��������Ĳ���
															(UBaseType_t 	)VCU_BRAKE_Pr, 				         //�������ȼ�
															(TaskHandle_t*  )&VCU_Brake_Task_Handle);	   //������ƿ�ָ��   
	
	if(pdPASS == xReturn)/* �����ɹ� */
		printf("VCU_BRAKE_TASK���񴴽��ɹ�!\r\n");
	else
		printf("VCU_BRAKE_TASK���񴴽�ʧ��!\r\n");

	  /* ����VCU_COMMAND_SEND���� */
	VCU_COMMAND_Task_Handle = xTaskCreateStatic((TaskFunction_t	)VCU_COMMAND_Send,//������
															(const char* 	)"VCU_COMMAND_Send",		            //��������
															(uint32_t 		)VCU_COMMAND_Stack,					                      //�����ջ��С
															(void* 		  	)NULL,				                      //���ݸ��������Ĳ���
															(UBaseType_t 	)VCU_COMMAND_Pr, 				                        //�������ȼ�
															(StackType_t*   )VCU_COMMAND_Task_Stack,	        //�����ջ
															(StaticTask_t*  )&VCU_COMMAND_Task_TCB);	        //������ƿ�  
	
	if(NULL != VCU_COMMAND_Task_Handle)                                                   /* �����ɹ� */
		printf("VCU_COMMAND_Send���񴴽��ɹ�!\r\n");
	else
		printf("VCU_COMMAND_Send���񴴽�ʧ��!\r\n");
	
	  /* ����MCU_STATUS1_Show���� */
	xReturn = xTaskCreate((TaskFunction_t	)MCU_STATUS1_Show,		               //������
															(const char* 	)"MCU_STATUS1_Show",		         //��������
															(uint16_t 		)MCU_STATUS1_Stack,					     //�����ջ��С
															(void* 		  	)NULL,  				               //���ݸ��������Ĳ���
															(UBaseType_t 	)MCU_STATUS1_Pr, 				         //�������ȼ�
															(TaskHandle_t*  )&MCU_STATUS1_Task_Handle);	   //������ƿ�ָ��   
	
	if(pdPASS == xReturn)/* �����ɹ� */
		printf("MCU_STATUS1_Show���񴴽��ɹ�!\r\n");
	else
		printf("MCU_STATUS1_Show���񴴽�ʧ��!\r\n");
	
	  /* ����MCU_STATUS2_Show���� */
	xReturn = xTaskCreate((TaskFunction_t	)MCU_STATUS2_Show,		               //������
															(const char* 	)"MCU_STATUS2_Show",		         //��������
															(uint16_t 		)MCU_STATUS2_Stack,					     //�����ջ��С
															(void* 		  	)NULL,  				               //���ݸ��������Ĳ���
															(UBaseType_t 	)MCU_STATUS2_Pr, 				         //�������ȼ�
															(TaskHandle_t*  )&MCU_STATUS2_Task_Handle);	   //������ƿ�ָ��   
	
	if(pdPASS == xReturn)/* �����ɹ� */
		printf("MCU_STATUS2_Show���񴴽��ɹ�!\r\n");
	else
		printf("MCU_STATUS2_Show���񴴽�ʧ��!\r\n");
	
	
	  /* ����MCU_STATUS3_Show���� */
	xReturn = xTaskCreate((TaskFunction_t	)MCU_STATUS3_Show,		               //������
															(const char* 	)"MCU_STATUS3_Show",		         //��������
															(uint16_t 		)MCU_STATUS3_Stack,					     //�����ջ��С
															(void* 		  	)NULL,  				               //���ݸ��������Ĳ���
															(UBaseType_t 	)MCU_STATUS3_Pr, 				         //�������ȼ�
															(TaskHandle_t*  )&MCU_STATUS3_Task_Handle);	   //������ƿ�ָ��   
	
	if(pdPASS == xReturn)/* �����ɹ� */
		printf("MCU_STATUS3_Show���񴴽��ɹ�!\r\n");
	else
		printf("MCU_STATUS3_Show���񴴽�ʧ��!\r\n");	
	
	  /* ����MCU_STATUS4_Show���� */
	xReturn = xTaskCreate((TaskFunction_t	)MCU_STATUS4_Show,		               //������
															(const char* 	)"MCU_STATUS4_Show",		         //��������
															(uint16_t 		)MCU_STATUS4_Stack,					     //�����ջ��С
															(void* 		  	)NULL,  				               //���ݸ��������Ĳ���
															(UBaseType_t 	)MCU_STATUS4_Pr, 				         //�������ȼ�
															(TaskHandle_t*  )&MCU_STATUS4_Task_Handle);	   //������ƿ�ָ��   
	
	if(pdPASS == xReturn)/* �����ɹ� */
		printf("MCU_STATUS4_Show���񴴽��ɹ�!\r\n");
	else
		printf("MCU_STATUS4_Show���񴴽�ʧ��!\r\n");	
	
		  /* ����CAN_RX_HANDLE���� */
	xReturn = xTaskCreate((TaskFunction_t	)CAN_RX_HANDLE,		                 //������
															(const char* 	)"CAN_RX_HANDLE",		           //��������
															(uint16_t 		)CAN_RX_HANDLE_Stack,					 //�����ջ��С
															(void* 		  	)NULL,  				               //���ݸ��������Ĳ���
															(UBaseType_t 	)CAN_RX_HANDLE_Pr, 				     //�������ȼ�
															(TaskHandle_t*  )&CAN_RX_HANDLE_Task_Handle);	   //������ƿ�ָ��   
	
	if(pdPASS == xReturn)/* �����ɹ� */
		printf("CAN_RX_HANDLE���񴴽��ɹ�!\r\n");
	else
		printf("CAN_RX_HANDLE���񴴽�ʧ��!\r\n");	
	
		  /* ����Dangwei_Scan���� */
	xReturn = xTaskCreate((TaskFunction_t	)Dangwei_Scan,		                 //������
															(const char* 	)"Dangwei_Scan",		           //��������
															(uint16_t 		)Dangwei_Scan_Stack,					 //�����ջ��С
															(void* 		  	)NULL,  				               //���ݸ��������Ĳ���
															(UBaseType_t 	)Dangwei_Scan_Pr, 				     //�������ȼ�
															(TaskHandle_t*  )&Dangwei_Scan_Task_Handle);	   //������ƿ�ָ��   
	
	if(pdPASS == xReturn)/* �����ɹ� */
		printf("Dangwei_Scan���񴴽��ɹ�!\r\n");
	else
		printf("Dangwei_Scan���񴴽�ʧ��!\r\n");

		  /* ����Dangwei_HANDLE���� */
	xReturn = xTaskCreate((TaskFunction_t	)Dangwei_HANDLE,		                 //������
															(const char* 	)"Dangwei_HANDLE",		           //��������
															(uint16_t 		)Dangwei_HANDLE_Stack,					 //�����ջ��С
															(void* 		  	)NULL,  				               //���ݸ��������Ĳ���
															(UBaseType_t 	)Dangwei_HANDLE_Pr, 				     //�������ȼ�
															(TaskHandle_t*  )&Dangwei_HANDLE_Task_Handle);	   //������ƿ�ָ��   
	
	if(pdPASS == xReturn)/* �����ɹ� */
		printf("Dangwei_HANDLE���񴴽��ɹ�!\r\n");
	else
		printf("Dangwei_HANDLE���񴴽�ʧ��!\r\n");			
	
		  /* ����Wit_Dat_HANDLE���� */
	xReturn = xTaskCreate((TaskFunction_t	)Wit_Dat_HANDLE,		                 //������
															(const char* 	)"Wit_Dat_HANDLE",		           //��������
															(uint16_t 		)Wit_Dat_HANDLE_Stack,					 //�����ջ��С
															(void* 		  	)NULL,  				               //���ݸ��������Ĳ���
															(UBaseType_t 	)Wit_Dat_HANDLE_Pr, 				     //�������ȼ�
															(TaskHandle_t*  )&Wit_Dat_HANDLE_Task_Handle);	   //������ƿ�ָ��   
	
	if(pdPASS == xReturn)/* �����ɹ� */
		printf("Wit_Dat_HANDLE���񴴽��ɹ�!\r\n");
	else
		printf("Wit_Dat_HANDLE���񴴽�ʧ��!\r\n");		
	
  vTaskDelete(AppTaskCreate_Handle);                                            //ɾ��AppTaskCreate����
    
  taskEXIT_CRITICAL();                                                          //�˳��ٽ���
}




/**********************************************************************
  * @ API  �� VCU_COMMAND_Send
  * @ brief�� VCU_COMMAND_Send��������
  * @ param    ��None   
  * @ retval  �� None
  ********************************************************************/

static void VCU_COMMAND_Send(void* parameter) //VCU_COMMAND����
{
	while(1)
	{
		xSemaphoreTake(BinarySem1_Handle,portMAX_DELAY);
			/* ��ʼ��������� */
		vcu_cmd.MCU_Enable = 1;
		vcu_cmd.Fault_Reset = 0;
		vcu_cmd.Control_Mode = Speed_Control;
		if(Gears_status == 1)
		{
			vcu_cmd.Demand_Limit_High = D_Torque_Limit_High;    //ǰ��ת������
			vcu_cmd.Demand_Limit_Low = D_Torque_Limit_Low; 
		}
		else if(Gears_status == 2)
		{
			vcu_cmd.Demand_Limit_High = R_Torque_Limit_High;    //����ת������
			vcu_cmd.Demand_Limit_Low = R_Torque_Limit_Low;
		}
    //ת������
		vcu_cmd.Live_Counter = 0;
		vcu_cmd.Demand_Torque = 0;
		ADC_Get_Throttle_Signal();
		#if USRAT_Memory_Throttle_Signal
		printf("Memory_Throttle_Signal = %d\r\n",Memory_Throttle_Signal);
		printf("Memory_Throttle_Signal = %d\r\n",-Memory_Throttle_Signal);		
		#endif
		#if USART_Throttle_Signal_1
		printf("Throttle_Signal = %d\r\n",Throttle_Signal);
		#endif
		LINEAR_SPEED_DEAL();					
		vTaskDelay(VCU_COMMAND_Delay);
	}
}

/**********************************************************************
  * @ ������  �� MCU_STATUS1_Show
  * @ ����˵���� ����MCU_STATUS1ָ���
  * @ ����    ��   
  * @ ����ֵ  �� ��
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
			/* MCU_STATUS1����������ݽ��� */
			res = MCU_STATUS1_ParseData(&mcu_sta1, Status1_Buf);
			if(res == 0)/* �ӹ���� */
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
  * @ API  �� MCU_STATUS2_Show
  * @ brief�� ����MCU_STATUS2ָ���
  * @ param    �� None  
  * @ retval  �� None
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
			/* MCU_STATUS1����������ݽ��� */
			res = MCU_STATUS2_ParseData(&mcu_sta2, Status2_Buf);
			if(res == 0)/* �ӹ���� */
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
  * @ API  �� MCU_STATUS3_Show
  * @ brief�� ����MCU_STATUS4ָ���
  * @ param   �� None
  * @ retval  �� None
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
			/* MCU_STATUS1����������ݽ��� */
			res = MCU_STATUS3_ParseData(&mcu_sta3, Status3_Buf);
			if(res == 0)/* �ӹ���� */
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
  * @ API  �� MCU_STATUS4_Show
  * @ brief�� ����MCU_STATUS4ָ���
  * @ param   ��   
  * @ retval  �� None
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
			/* MCU_STATUS1����������ݽ��� */
			res = MCU_STATUS4_ParseData(&mcu_sta4, Status4_Buf);
			if(res == 0)/* �ӹ���� */
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
  * @ API  �� VCU_Brake_Task
  * @ brief�� �ж��ƶ�̤��Һѹ��������ֵ
  * @ param�� None
  * @ retval��None
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
			vcu_cmd.MCU_Enable = 0; //�����������ʹ��
			res = VCU_COMMAND_SendData_Process(vcu_cmd, VCU_Command_Send);
			if(res == 0) /* �ӹ���� */
			{
				res = Can_Send_Msg(VCU_Command, VCU_Command_Send, 8);//����8���ֽ�
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
  * @ API  �� CAN_RX_HANDLE
  * @ brief�� ����CAN���ܵı���
  * @ param�� None 
  * @ retval��None
  ********************************************************************/
static void CAN_RX_HANDLE(void* parameter)
{
	uint8_t i = 0;
	while(1)
	{
  	xSemaphoreTake(BinarySem2_Handle,portMAX_DELAY);
				/* ��ʼ��CAN������ */
		RxMessage.StdId=0x00;
		RxMessage.ExtId=0x00;
		RxMessage.IDE=0;
		RxMessage.DLC=0;
		RxMessage.FMI=0;
		for(i = 0; i < 8; i++)
		{
			RxMessage.Data[i]=0x00;
		}
		/* �������� */
		CAN_Receive(CANx, CAN_FIFO0, &RxMessage);
		/* �ж��Ƿ�ΪĿ�귽����,�������ݽ���У�� */	
		if((RxMessage.ExtId==MCU_Status1) && (RxMessage.IDE==CAN_Id_Extended) && (RxMessage.DLC==8))
		{
			/*�¼���λ*/
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
		//����CAN����
		if((RxMessage.StdId==Wit_dat) && (RxMessage.IDE==CAN_Id_Standard) && (RxMessage.DLC==8))
		{
			WitCanDataIn(RxMessage.Data, RxMessage.DLC);
			xEventGroupSetBits(EventGroupHandler,EVENTBIT_6);
		}
	}
}



/**********************************************************************
  * @ API  �� LINEAR_SPEED_DEAL
  * @ brief�� ����adc���ݣ��жϼ��٣�����
  * @ param�� None 
  * @ retval��None
  ********************************************************************/
static __INLINE void LINEAR_SPEED_DEAL(void)
{
	 uint8_t res = 0;
	 int difference_value = 0;
	 uint8_t Motor_status = 0;
	 difference_value = Memory_Throttle_Signal - Throttle_Signal;
	 

	 if(difference_value > Sensitivity){Motor_status = 1;}                   //����
	 else if(difference_value + Sensitivity >= 0&&difference_value <= Sensitivity){Motor_status = 2;}//����
	 else if(difference_value + Sensitivity< 0){Motor_status = 3;}           //����
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
							if(res == 0) /* �ӹ���� */
							{
								res = Can_Send_Msg(VCU_Command, VCU_Command_Send, 8);//����8���ֽ�
							}
							vTaskDelay(15);
							goto Disable_Speed_flag;
						}
						if(Gears_status == 1)
						{
							vcu_cmd.Demand_Speed = Throttle_Signal;
						}
						else if(Gears_status == 2)
						{
							vcu_cmd.Demand_Speed = -Throttle_Signal;
						}
						vTaskDelay(50);                                                //����ֵ
					}
					Disable_Speed_flag = 0;
				}
				else
				{
	        if(Throttle_Signal>=Disable_Speed)
					{
						if(Gears_status == 1)
						{
							vcu_cmd.Demand_Speed = Throttle_Signal;							
						}
						else if(Gears_status == 2)
						{
							vcu_cmd.Demand_Speed = -Throttle_Signal;
						}	
					}
					else
					{
						//���磬�����Ϊ�յ�
						Memory_Throttle_Signal = 0;
						vcu_cmd.MCU_Enable = 0;                                          //�����������ʹ��
					}
				}
				res = VCU_COMMAND_SendData_Process(vcu_cmd, VCU_Command_Send);
				if(res == 0) /* �ӹ���� */
				{
					res = Can_Send_Msg(VCU_Command, VCU_Command_Send, 8);//����8���ֽ�
				}
			  break;
			}
		 case(2):
			{
				if(Memory_Throttle_Signal > Throttle_Signal)
				{
					 if(Memory_Throttle_Signal >= Disable_Speed)
					 {
							if(Gears_status == 1)
							{
								vcu_cmd.Demand_Speed = Memory_Throttle_Signal;
								//printf("%d",Memory_Throttle_Signal);
							}
							else if(Gears_status == 2)
							{
								vcu_cmd.Demand_Speed = -Memory_Throttle_Signal;
								//printf("%d",Memory_Throttle_Signal*-1);
							}
					 }
					 else
					 {	//���磬�����Ϊ�յ�
						 	Memory_Throttle_Signal = 0;
							vcu_cmd.MCU_Enable = 0;       //�����������ʹ��
					 }
					 res = VCU_COMMAND_SendData_Process(vcu_cmd, VCU_Command_Send);
					 if(res == 0) /* �ӹ���� */
					 {
							res = Can_Send_Msg(VCU_Command, VCU_Command_Send, 8);//����8���ֽ�
					 }
				}
				else
				{
					 if(Throttle_Signal >= Disable_Speed)
					 {
						  if(Gears_status == 1)
							{
								vcu_cmd.Demand_Speed = Throttle_Signal;							
							}
							else if(Gears_status == 2)
							{
								vcu_cmd.Demand_Speed = -Throttle_Signal;
							}
					 }
					 else
					 {
						 Memory_Throttle_Signal = 0;
						 vcu_cmd.MCU_Enable = 0;       //�����������ʹ��
					 }
					 res = VCU_COMMAND_SendData_Process(vcu_cmd, VCU_Command_Send);
					 if(res == 0) /* �ӹ���� */
					 {
							res = Can_Send_Msg(VCU_Command, VCU_Command_Send, 8);//����8���ֽ�
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
						if(Gears_status == 1)
						{
							vcu_cmd.Demand_Speed = mcu_sta1.Motor_Speed;							
						}
						else if(Gears_status == 2)
						{
							vcu_cmd.Demand_Speed = -mcu_sta1.Motor_Speed;
						}
					}
					else
					{
						if(Gears_status == 1)
						{
							vcu_cmd.Demand_Speed = Throttle_Signal;     //Memory_Throttle_Signal							
						}
						else if(Gears_status == 2)
						{
							vcu_cmd.Demand_Speed = -Throttle_Signal;
						}						
					}
				}
				else
				{
					 Disable_Speed_flag = 1;
					 Memory_Throttle_Signal = 0;
					 vcu_cmd.MCU_Enable = 0;       //�����������ʹ��
				}
				res = VCU_COMMAND_SendData_Process(vcu_cmd, VCU_Command_Send);
				if(res == 0) /* �ӹ���� */
				{
						res = Can_Send_Msg(VCU_Command, VCU_Command_Send, 8);//����8���ֽ�
				}
				break;
			}
		 default:
		break;
	 }
 }


/**********************************************************************
  * @ API  �� ADC_Get_Throttle_Signal
  * @ brief�� �����������
  * @ param   �� None 
  * @ retval  �� None
  ********************************************************************/

static __INLINE void ADC_Get_Throttle_Signal(void) //ADC��ȡ�����ź�
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
		if(Gears_status == 1 || Gears_status == 2)
		{
			Throttle_Signal = adc*(Motorspeed*1.0/3300);
		}
		else{Throttle_Signal = 0;}
	}
}



/**********************************************************************
  * @ API  �� Wit_Dat_HANDLE
  * @ brief�� ������ᴫ�н��ܵı���
  * @ param�� None 
  * @ retval��None
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


/**********************************************************************
  * @ API  �� Dangwei_Scan
  * @ brief�� ������������λ�л�ɨ��
  * @ param�� None 
  * @ retval��None
  ********************************************************************/
static void Dangwei_Scan(void* parameter)
{
	uint8_t key_value = 0xff;
	static uint8_t daishi_flag = 0;
	while(1)
	{
//		xSemaphoreTake(MuxSem_Handle,portMAX_DELAY);
		key_value = Key_scan();
		if(key_value == 1 && brake_flag == 1  )               
		{
			daishi_flag = 1;
			printf("1\r\n");
		}	
		if(daishi_flag == 1)
		{
			if(key_value == 2)          //ǰ����
			{
//				xSemaphoreGive( MuxSem_Handle );             //����������
				Gears_flag = 0xa;
				xSemaphoreGive(BinarySem3_Handle);
				printf("2\r\n");
			}
			else if(key_value == 3)          //���˵�
			{
//				xSemaphoreGive( MuxSem_Handle );             //����������	
				Gears_flag = 0xb;
				xSemaphoreGive(BinarySem3_Handle);
				printf("3\r\n");
			}
		}
		vTaskDelay(Dangwei_HANDLE_Delay);
	}
}

/**********************************************************************
  * @ API  �� Dangwei_HANDLE
  * @ brief�� ������������λ�л�������
  * @ param�� None 
  * @ retval��None
  ********************************************************************/
static void Dangwei_HANDLE(void* parameter)
{
	uint8_t res = 0;
	while(1)
	{
		xSemaphoreTake(BinarySem3_Handle,portMAX_DELAY);		//MuxSem_Handle
		printf("Gears_flag = %d\r\n",Gears_flag);
		if(Gears_flag == 0xa)
		{
			vcu_cmd2.Gears_Signal = Gears_D;
			Gears_status = 1;printf("ǰ��%d\r\n",Gears_status);
		}
		else if(Gears_flag == 0xb)
		{
			vcu_cmd2.Gears_Signal = Gears_R;
			Gears_status = 2;printf("����%d\r\n",Gears_status);
		}
		Gears_flag = 0;
		res = VCU_COMMAND2_SendData_Process(vcu_cmd2, VCU_Command_Send);
		if(res == 0) /* �ӹ���� */
		{
			res = Can_Send_Msg(VCU_Command, VCU_Command_Send, 8);//����8���ֽ�
		}
		vTaskDelay(Dangwei_Scan_Delay);
	}
}


