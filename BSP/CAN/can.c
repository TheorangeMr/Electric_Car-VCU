#include "can.h"


extern CanRxMsg RxMessage;
extern SemaphoreHandle_t  BinarySem2_Handle;

/**************************CAN���ú���**************************/
/*
  ��������CAN_GPIO_Config()
  ��  �ܣ�CAN�������ú���
  ��  ������
	����ֵ����
*/
static void CAN_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* ʹ��CAN��TX��RX����ʱ�� */
	RCC_APB2PeriphClockCmd(CAN_RX_CLK|CAN_TX_CLK, ENABLE);
	
	/* CAN_RX��������Ϊ������������������ */
	GPIO_InitStructure.GPIO_Pin = CAN_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(CAN_RX_PORT, &GPIO_InitStructure);
	
	/* CAN_TX��������Ϊ����������� */
	GPIO_InitStructure.GPIO_Pin = CAN_TX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(CAN_TX_PORT, &GPIO_InitStructure);
}

/*
	��������CAN_NVIC_Config()
	��  �ܣ�CAN�ж����ú���
	��  ������
	����ֵ����
*/
static void CAN_NVIC_Config(void)
{	
	NVIC_InitTypeDef NVIC_InitStructure;
	
//	/* ����Ϊ�ж����ȼ�����1 */
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	/* �ж����� */
	NVIC_InitStructure.NVIC_IRQChannel = CAN_RX_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  // �����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         // �����ȼ�Ϊ0
	
	NVIC_Init(&NVIC_InitStructure);
}

/*
	��������CAN_Mode_Config()
	��  �ܣ�CANģʽ���ú���
	��  ������
	����ֵ����
*/
static void CAN_Mode_Config(void)
{
	CAN_InitTypeDef CAN_InitStructure;
	
	RCC_APB1PeriphClockCmd(CAN_CLK, ENABLE);
	
	/* CAN register init */
  CAN_DeInit(CANx);
  CAN_StructInit(&CAN_InitStructure);

  /* CAN��Ԫ���� */
  CAN_InitStructure.CAN_TTCM=DISABLE;           //�ر�ʱ�䴥��ͨ��ģʽ
  CAN_InitStructure.CAN_ABOM=DISABLE;           //����Զ����߹���
  CAN_InitStructure.CAN_AWUM=DISABLE;           //�ر��Զ�����ģʽ��ͨ���������(���CAN->MCR��SLEEPλ)
  CAN_InitStructure.CAN_NART=DISABLE;           //�رս�ֹ�Զ��ش������������Զ����ͣ�
  CAN_InitStructure.CAN_RFLM=DISABLE;           //���Ĳ�����,�µĸ��Ǿɵ�
  CAN_InitStructure.CAN_TXFP=DISABLE;           //���ȼ��ɱ��ı�ʶ������
  CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;   //����ģʽ
	
  /*
    ���ò����� = 250Kbps
    brp :�����ʷ�Ƶ��.��Χ:1~1024;  tq=(brp)*tpclk1
    ������ = Fpclk1/((tbs1+1+tbs2+1+1)*brp)
    Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ36M���������CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_13tq, CAN_Prescaler = 9;
    ������Ϊ:36M/((1+2+13)*9)=250Kbps
  */
  CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;  //����ͬ����Ծʱ�䵥Ԫ:Tsjw = tsjw+1��ʱ�䵥λ CAN_SJW_1tq ~ CAN_SJW_4tq
  CAN_InitStructure.CAN_BS1=CAN_BS1_13tq; //ʱ��� 1 ��ʱ�䵥Ԫ :Tbs1 = tbs1+1��ʱ�䵥λ CAN_BS1_1tq ~ CAN_BS1_16tq
  CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;  //ʱ��� 2 ��ʱ�䵥Ԫ :Tbs2 = tbs2+1��ʱ�䵥λ CAN_BS2_1tq ~ CAN_BS2_8tq
  CAN_InitStructure.CAN_Prescaler=9;      //��Ƶϵ��(Fdiv)Ϊbrp+1
  CAN_Init(CANx, &CAN_InitStructure);     //��ʼ��CANx
}

/*
	��������CAN_Filter_Config()
	��  �ܣ�CANɸѡ�����ú���
	��  ������
	����ֵ����
*/
static void CAN_Filter_Config(void)
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	
	/* CAN filter init */
  CAN_FilterInitStructure.CAN_FilterNumber=0;                   //ɸѡ����0
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; //����λģʽ
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;//32λɸѡ��
  CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;              //ɸѡ����16λ
  CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;               //ɸѡ����16λ
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;          //ɸѡ����16λ������
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;           //ɸѡ����16λ������
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;           //ɸѡ��������FIFO0
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;          //ʹ��ɸѡ��
	
  CAN_FilterInit(&CAN_FilterInitStructure);                     //�˲�����ʼ��
	
	/* FIFO0��Ϣ�Һ��ж����� */ 
  CAN_ITConfig(CANx, CAN_IT_FMP0, DISABLE);
}

/**************************CAN���⺯���ӿ�**************************/
/*
	��������CAN_Config()
	��  �ܣ�CAN���ú���
	��  ������
	����ֵ����
*/
void CAN_Config(void)
{
	CAN_GPIO_Config();
	CAN_NVIC_Config();
	CAN_Mode_Config();
	CAN_Filter_Config();
}

/*
	��������Can_Send_Msg()
	��  �ܣ�CAN����֡����
  ��  ����u8 Ext_ID����չ��ʶ��, u8* msg������ָ��, u8 len�����ݳ���
  ����ֵ��PASSED:����ɹ���FAILED:ʧ��
*/
u8 Can_Send_Msg(u32 Ext_ID,u8* msg,u8 len)
{	
  u8 mbox;
  u32 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0;					   //��׼��ʶ�� 
  TxMessage.ExtId=Ext_ID;	       //������չ��ʾ�� 
  TxMessage.IDE=CAN_Id_Extended; //��չ֡
  TxMessage.RTR=CAN_RTR_Data;		 //����֡
  TxMessage.DLC=len;						 //Ҫ���͵����ݳ���
	
	/* ����������д�뷢������ */
  for(i = 0; i < len; i++)
	{
		TxMessage.Data[i] = msg[i];	
	}         
  mbox= CAN_Transmit(CANx, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CANx, mbox) == CAN_TxStatus_Failed)&&(i < 0XFFF)) 
		i++;	//�ȴ����ͽ���
  if(i >= 0XFFF) //����ɹ�
		return 0;
	else
		return 1;//����ʧ��
}

/*
	��������CAN_RX_IRQHandler(void)
	��  �ܣ�CAN�жϽ��պ���
	��  ������
  ����ֵ����
*/
#if CAN_RX0_INT_ENABLE == 1	//ʹ��RX0�ж�
void CAN_RX_IRQHandler(void)
{
	u8 i = 0;
	BaseType_t pxHigherPriorityTaskWoken,xResult;
	if(CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
//		printf("Interrupt OK!\r\n");
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
	��������Can_Receive_Msg()
	��  �ܣ�CAN����֡����
	��  ����u8* msg������ָ��, u8 len�����ݳ���
  ����ֵ��PASSED:����ɹ���FAILED:ʧ��
*/
#if CAN_RX0_INT_ENABLE == 0	//��ʹ��RX0�ж�
u8 Can_Receive_Msg(u8 *msg,u8 len)
{		   		   
 	u32 i;
	u8 u;
	CanRxMsg RxMessage;
	
	/* �ȴ�������𣨽��յ���Ϣ�� */
	i = 0;
  while((CAN_MessagePending(CANx, CAN_FIFO0) < 1) && (i != 0xFFFF)) i++;
	if(i ==0xFF) //û�н��յ�����
		return FAILED;
	else
	{
		/* ��ʼ��CAN������ */
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
	/* �ж��Ƿ�ΪĿ�귽����,�������ݽ���У�� */
  if((RxMessage.ExtId==0x0CFF0008) && (RxMessage.IDE==CAN_Id_Extended) && (RxMessage.DLC==len))
	{
		return PASSED;//����ɹ�
	}
	else
	{
		return FAILED;//����ʧ��
	}
}
#endif









