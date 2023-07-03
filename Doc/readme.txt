2022.10.5
1.任务堆栈与任务创建时不一致，导致程序进入硬件中断。
2.任务无法调度是因为任务切换缺少任务切换函数，导致程序移植运行在当前任务。
2022.10.15
创建MCU状态读取进程，用于串口调试，以及后期用于仪表显示。


2023.1.30
更换电卡丁逻辑程序，使其适用于电巴哈。

vTaskDelay:
#define   VCU_COMMAND_Delay             7                      //MCU STATUS1的电机控制器发送周期为10sm
#define   VCU_BRAKE_Delay               VCU_COMMAND_Delay*2    
#define   MCU_STATUS2_Delay							399                    //MCU STATUS2     20sm
#define   MCU_STATUS3_Delay							299                    //MCU STATUS3     20sm
#define   MCU_STATUS4_Delay							199                    //MCU STATUS4     50sm
#define   Wit_collect_Delay                         200	

2023.7.2
移植九轴传感器SDK，通过CAN通讯采集三轴加速度
添加九轴采集线程


