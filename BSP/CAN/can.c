#include "can.h"


extern CanRxMsg RxMessage;
extern SemaphoreHandle_t  BinarySem2_Handle;

/**************************CAN配置函数**************************/
/*
  函数名：CAN_GPIO_Config()
  功  能：CAN引脚配置函数
  参  数：无
	返回值：无
*/
static void CAN_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* 使能CAN的TX、RX引脚时钟 */
	RCC_APB2PeriphClockCmd(CAN_RX_CLK|CAN_TX_CLK, ENABLE);
	
	/* CAN_RX引脚配置为浮空输入或带上拉输入 */
	GPIO_InitStructure.GPIO_Pin = CAN_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(CAN_RX_PORT, &GPIO_InitStructure);
	
	/* CAN_TX引脚配置为复用推挽输出 */
	GPIO_InitStructure.GPIO_Pin = CAN_TX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(CAN_TX_PORT, &GPIO_InitStructure);
}

/*
	函数名：CAN_NVIC_Config()
	功  能：CAN中断配置函数
	参  数：无
	返回值：无
*/
static void CAN_NVIC_Config(void)
{	
	NVIC_InitTypeDef NVIC_InitStructure;
	
//	/* 配置为中断优先级分组1 */
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	/* 中断配置 */
	NVIC_InitStructure.NVIC_IRQChannel = CAN_RX_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  // 主优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         // 次优先级为0
	
	NVIC_Init(&NVIC_InitStructure);
}

/*
	函数名：CAN_Mode_Config()
	功  能：CAN模式配置函数
	参  数：无
	返回值：无
*/
static void CAN_Mode_Config(void)
{
	CAN_InitTypeDef CAN_InitStructure;
	
	RCC_APB1PeriphClockCmd(CAN_CLK, ENABLE);
	
	/* CAN register init */
  CAN_DeInit(CANx);
  CAN_StructInit(&CAN_InitStructure);

  /* CAN单元设置 */
  CAN_InitStructure.CAN_TTCM=DISABLE;           //关闭时间触发通信模式
  CAN_InitStructure.CAN_ABOM=DISABLE;           //软件自动离线管理
  CAN_InitStructure.CAN_AWUM=DISABLE;           //关闭自动唤醒模式，通过软件唤醒(清除CAN->MCR的SLEEP位)
  CAN_InitStructure.CAN_NART=DISABLE;           //关闭禁止自动重传（开启报文自动传送）
  CAN_InitStructure.CAN_RFLM=DISABLE;           //报文不锁定,新的覆盖旧的
  CAN_InitStructure.CAN_TXFP=DISABLE;           //优先级由报文标识符决定
  CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;   //正常模式
	
  /*
    设置波特率 = 250Kbps
    brp :波特率分频器.范围:1~1024;  tq=(brp)*tpclk1
    波特率 = Fpclk1/((tbs1+1+tbs2+1+1)*brp)
    Fpclk1的时钟在初始化的时候设置为36M，如果设置CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_13tq, CAN_Prescaler = 9;
    则波特率为:36M/((1+2+13)*9)=250Kbps
  */
  CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;  //重新同步跳跃时间单元:Tsjw = tsjw+1个时间单位 CAN_SJW_1tq ~ CAN_SJW_4tq
  CAN_InitStructure.CAN_BS1=CAN_BS1_13tq; //时间段 1 的时间单元 :Tbs1 = tbs1+1个时间单位 CAN_BS1_1tq ~ CAN_BS1_16tq
  CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;  //时间段 2 的时间单元 :Tbs2 = tbs2+1个时间单位 CAN_BS2_1tq ~ CAN_BS2_8tq
  CAN_InitStructure.CAN_Prescaler=9;      //分频系数(Fdiv)为brp+1
  CAN_Init(CANx, &CAN_InitStructure);     //初始化CANx
}

/*
	函数名：CAN_Filter_Config()
	功  能：CAN筛选器配置函数
	参  数：无
	返回值：无
*/
static void CAN_Filter_Config(void)
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	
	/* CAN filter init */
  CAN_FilterInitStructure.CAN_FilterNumber=0;                   //筛选器组0
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; //屏蔽位模式
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;//32位筛选器
  CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;              //筛选器高16位
  CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;               //筛选器低16位
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;          //筛选器高16位不关心
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;           //筛选器低16位不关心
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;           //筛选器关联到FIFO0
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;          //使能筛选器
	
  CAN_FilterInit(&CAN_FilterInitStructure);                     //滤波器初始化
	
	/* FIFO0消息挂号中断允许 */ 
  CAN_ITConfig(CANx, CAN_IT_FMP0, DISABLE);
}

/**************************CAN对外函数接口**************************/
/*
	函数名：CAN_Config()
	功  能：CAN配置函数
	参  数：无
	返回值：无
*/
void CAN_Config(void)
{
	CAN_GPIO_Config();
	CAN_NVIC_Config();
	CAN_Mode_Config();
	CAN_Filter_Config();
}

/*
	函数名：Can_Send_Msg()
	功  能：CAN发送帧函数
  参  数：u8 Ext_ID：扩展标识符, u8* msg：数据指针, u8 len：数据长度
  返回值：PASSED:传输成功，FAILED:失败
*/
u8 Can_Send_Msg(u32 Ext_ID,u8* msg,u8 len)
{	
  u8 mbox;
  u32 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0;					   //标准标识符 
  TxMessage.ExtId=Ext_ID;	       //设置扩展标示符 
  TxMessage.IDE=CAN_Id_Extended; //扩展帧
  TxMessage.RTR=CAN_RTR_Data;		 //数据帧
  TxMessage.DLC=len;						 //要发送的数据长度
	
	/* 将发送数据写入发送邮箱 */
  for(i = 0; i < len; i++)
	{
		TxMessage.Data[i] = msg[i];	
	}         
  mbox= CAN_Transmit(CANx, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CANx, mbox) == CAN_TxStatus_Failed)&&(i < 0XFFF)) 
		i++;	//等待发送结束
  if(i >= 0XFFF) //传输成功
		return 0;
	else
		return 1;//传输失败
}


/*
	函数名：Wit_Can_Send_Msg()
	功  能：CAN发送帧函数
  参  数：u8 ucStdId：扩展标识符, u8* msg：数据指针, u8 len：数据长度
  返回值：PASSED:传输成功，FAILED:失败
*/
void Wit_Can_Send_Msg(uint8_t ucStdId, uint8_t* msg, uint32_t len)
{
	uint8_t mbox;
	uint16_t i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=ucStdId;			
	TxMessage.ExtId=0;			
	TxMessage.IDE=CAN_Id_Standard; 
	TxMessage.RTR=CAN_RTR_Data;		
	TxMessage.DLC=len;			
	for(i=0;i<len;i++)
	TxMessage.Data[i]=msg[i];			          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0; 
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;
}



/*
	函数名：CAN_RX_IRQHandler(void)
	功  能：CAN中断接收函数
	参  数：无
  返回值：无
*/
#if CAN_RX0_INT_ENABLE == 1	//使能RX0中断
void CAN_RX_IRQHandler(void)
{
	u8 i = 0;
	BaseType_t pxHigherPriorityTaskWoken,xResult;
	if(CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
//		printf("Interrupt OK!\r\n");
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
		pxHigherPriorityTaskWoken = pdFALSE;
		xResult = xSemaphoreGiveFromISR(BinarySem2_Handle ,&pxHigherPriorityTaskWoken);
		if(xResult != pdFAIL)
		{
			portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		}
	}
}
#endif

/*
	函数名：Can_Receive_Msg()
	功  能：CAN接收帧函数
	参  数：u8* msg：数据指针, u8 len：数据长度
  返回值：PASSED:传输成功，FAILED:失败
*/
#if CAN_RX0_INT_ENABLE == 0	//不使能RX0中断
u8 Can_Receive_Msg(u8 *msg,u8 len)
{		   		   
 	u32 i;
	u8 u;
	CanRxMsg RxMessage;
	
	/* 等待邮箱挂起（接收到信息） */
	i = 0;
  while((CAN_MessagePending(CANx, CAN_FIFO0) < 1) && (i != 0xFFFF)) i++;
	if(i ==0xFF) //没有接收到数据
		return FAILED;
	else
	{
		/* 初始化CAN接收器 */
		RxMessage.ExtId=0x00;
		RxMessage.IDE=CAN_Id_Extended;
		RxMessage.DLC=0;
		for(u = 0; u < 8; u++)
		{
			RxMessage.Data[u]=0x00;
		}
		/* receive */
		CAN_Receive(CANx, CAN_FIFO0, &RxMessage);
	}
	/* 判断是否为目标方发送,并对数据进行校验 */
  if((RxMessage.ExtId==0x0CFF0008) && (RxMessage.IDE==CAN_Id_Extended) && (RxMessage.DLC==len))
	{
		return PASSED;//传输成功
	}
	else
	{
		return FAILED;//传输失败
	}
}
#endif









