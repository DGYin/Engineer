/**
 * @file drv_can.c
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的CAN通信初始化与配置流程
 * @version 0.1
 * @date 2022-08-02
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_can.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Struct_CAN_Manage_Object CAN1_Manage_Object = {0};
Struct_CAN_Manage_Object CAN2_Manage_Object = {0};

// CAN通信发送缓冲区
uint8_t CAN1_0x1ff_Tx_Data[8];
uint8_t CAN1_0x200_Tx_Data[8];
uint8_t CAN1_0x2ff_Tx_Data[8];
uint8_t CAN1_0xx01_Tx_Data[8];
uint8_t CAN1_0xx02_Tx_Data[8];
uint8_t CAN1_0xx11_Tx_Data[8];
uint8_t CAN1_0xx12_Tx_Data[8];
uint8_t CAN1_0xx13_Tx_Data[8];
uint8_t CAN1_0xx14_Tx_Data[8];
uint8_t CAN1_0xx15_Tx_Data[8];
uint8_t CAN1_0xx16_Tx_Data[8];

uint8_t CAN2_0x1ff_Tx_Data[8];
uint8_t CAN2_0x200_Tx_Data[8];
uint8_t CAN2_0x2ff_Tx_Data[8];

uint8_t CAN2_0xx11_Tx_Data[8];
uint8_t CAN2_0xx12_Tx_Data[8];
uint8_t CAN2_0xx13_Tx_Data[8];
uint8_t CAN2_0xx14_Tx_Data[8];
uint8_t CAN2_0xx15_Tx_Data[8];
uint8_t CAN2_0xx16_Tx_Data[8];

uint8_t CAN_Supercap_Tx_Data[8];

uint8_t CAN1_0xxf1_Tx_Data[8];
uint8_t CAN1_0xxf2_Tx_Data[8];
uint8_t CAN1_0xxf3_Tx_Data[8];
uint8_t CAN1_0xxf4_Tx_Data[8];
uint8_t CAN1_0xxf5_Tx_Data[8];
uint8_t CAN1_0xxf6_Tx_Data[8];
uint8_t CAN1_0xxf7_Tx_Data[8];
uint8_t CAN1_0xxf8_Tx_Data[8];

uint8_t CAN2_0xxf1_Tx_Data[8];
uint8_t CAN2_0xxf2_Tx_Data[8];
uint8_t CAN2_0xxf3_Tx_Data[8];
uint8_t CAN2_0xxf4_Tx_Data[8];
uint8_t CAN2_0xxf5_Tx_Data[8];
uint8_t CAN2_0xxf6_Tx_Data[8];
uint8_t CAN2_0xxf7_Tx_Data[8];
uint8_t CAN2_0xxf8_Tx_Data[8];
/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 配置CAN的过滤器
 *
 * @param hcan CAN编号
 * @param Object_Para 编号 | FIFOx | ID类型 | 帧类型
 * @param ID ID
 * @param Mask_ID 屏蔽位(0x3ff, 0x1fffffff)
 */
void can_filter_mask_config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
    CAN_FilterTypeDef can_filter_init_structure;

    //检测传参是否正确
    assert_param(hcan != NULL);

    if ((Object_Para & 0x02))
    {
        //数据帧
        //掩码后ID的高16bit
        can_filter_init_structure.FilterIdHigh = ID << 3 << 16;
        //掩码后ID的低16bit
        can_filter_init_structure.FilterIdLow = ID << 3 | ((Object_Para & 0x03) << 1);
        // ID掩码值高16bit
        can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 3 << 16;
        // ID掩码值低16bit
        can_filter_init_structure.FilterMaskIdLow = Mask_ID << 3 | ((Object_Para & 0x03) << 1);
    }
    else
    {
        //其他帧
        //掩码后ID的高16bit
        can_filter_init_structure.FilterIdHigh = ID << 5;
        //掩码后ID的低16bit
        can_filter_init_structure.FilterIdLow = ((Object_Para & 0x03) << 1);
        // ID掩码值高16bit
        can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 5;
        // ID掩码值低16bit
        can_filter_init_structure.FilterMaskIdLow = ((Object_Para & 0x03) << 1);
    }
    //滤波器序号, 0-27, 共28个滤波器, 前14个在CAN1, 后14个在CAN2
    can_filter_init_structure.FilterBank = Object_Para >> 3;
    //滤波器绑定FIFO0
    can_filter_init_structure.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;
    //使能滤波器
    can_filter_init_structure.FilterActivation = ENABLE;
    //滤波器模式，设置ID掩码模式
    can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
    // 32位滤波
    can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
    //从机模式选择开始单元
    can_filter_init_structure.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure);
}

/**
 * @brief 初始化CAN总线
 *
 * @param hcan CAN编号
 * @param Callback_Function 处理回调函数
 */
void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function)
{
    HAL_CAN_Start(hcan);
    

    if (hcan->Instance == CAN1)
    {
        CAN1_Manage_Object.CAN_Handler = hcan;
        CAN1_Manage_Object.Callback_Function = Callback_Function;
        can_filter_mask_config(hcan, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
        can_filter_mask_config(hcan, CAN_FILTER(1) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    }
    else if (hcan->Instance == CAN2)
    {
        CAN2_Manage_Object.CAN_Handler = hcan;
        CAN2_Manage_Object.Callback_Function = Callback_Function;
        can_filter_mask_config(hcan, CAN_FILTER(14) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
        can_filter_mask_config(hcan, CAN_FILTER(15) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    }
	
	/*开中断*/
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);	//开启CAN接收fifo0满中断
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);  //开启CAN接收fifo1满中断
}

/**
 * @brief 发送数据帧
 *
 * @param hcan CAN编号
 * @param ID ID
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @param __IDE 标准帧还是扩展帧
 * @return uint8_t 执行状态
 */
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length, uint32_t __IDE)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t used_mailbox;

    //检测传参是否正确
    assert_param(hcan != NULL);

	switch(__IDE)
	{
		case CAN_ID_STD:
			tx_header.StdId = ID;
			tx_header.ExtId = 0;
		break;
		case CAN_ID_EXT:
			tx_header.StdId = 0;
			tx_header.ExtId = ID;
		break;
	}
    tx_header.IDE = __IDE;
    tx_header.RTR = 0;
    tx_header.DLC = Length;
	
	while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0){}; // 等待邮箱清空
    return (HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox));
}

/**
 * @brief CAN的TIM定时器中断发送回调函数
 *
 */
void Task_CAN_PeriodElapsedCallback()
{
	static uint8_t send = 0;
//    static int mod10 = 0;
//    mod10++;
//    if (mod10 == 10 - 1)
//    {
//        mod10 = 0;
//        // CAN1超级电容
//        CAN_Send_Data(&hcan1, 0x220, CAN_Supercap_Tx_Data, 8);
//    }

    // CAN1电机
//    CAN_Send_Data(&hcan1, 0x1ff, CAN1_0x1ff_Tx_Data, 8, CAN_ID_STD);
	if(send == 0)
    {
		if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0)
		CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8, CAN_ID_STD);
//		CAN_Send_Data(&hcan1, 0xA1,  CAN1_0xxf1_Tx_Data, 8,CAN_ID_STD);

		send = 1;
	}
	else
	{
		if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0)
		CAN_Send_Data(&hcan1, 0x12, CAN1_0xx02_Tx_Data, 8, CAN_ID_STD);
		if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0)
		CAN_Send_Data(&hcan1, 0x11, CAN1_0xx01_Tx_Data, 8, CAN_ID_STD);
		send = 0;
	}
	
    
    // CAN_Send_Data(&hcan1, 0x2ff, CAN1_0x2ff_Tx_Data, 8);

    // CAN2电机
//    CAN_Send_Data(&hcan2, 0x1ff, CAN2_0x1ff_Tx_Data, 8, CAN_ID_STD);
//    CAN_Send_Data(&hcan2, 0x200, CAN2_0x200_Tx_Data, 8, CAN_ID_STD);
    // CAN_Send_Data(&hcan2, 0x2ff, CAN2_0x2ff_Tx_Data, 8);
}

/**
 * @brief HAL库CAN接收FIFO0中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    //判断程序初始化完成
//    if(init_finished == 0)
//    {
//        return;
//    }
    
	
    //选择回调函数
    //选择回调函数
    if (hcan->Instance == CAN1)
    {
		// 当CAN接收FIFO0中有数据时
		while(HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
        {
			// 从CAN1的FIFO0中获取接收消息
			HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CAN1_Manage_Object.Rx_Buffer.Header, CAN1_Manage_Object.Rx_Buffer.Data);
			// 调用CAN1管理对象的回调函数处理接收缓冲区数据
			CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
		}
    }
    else if (hcan->Instance == CAN2)
    {
		// 当CAN接收FIFO0中有数据时
		while(HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
        {
			HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CAN2_Manage_Object.Rx_Buffer.Header, CAN2_Manage_Object.Rx_Buffer.Data);
			CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
		}
    }
}

/**
 * @brief HAL库CAN接收FIFO1中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // //判断程序初始化完成, 出问题再加
    // if(init_finished == 0)
    // {
    //     return;
    // }
    
    //选择回调函数
    //选择回调函数
    if (hcan->Instance == CAN1)
    {
		// 当CAN接收FIFO1中有数据时
		while(HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) > 0)
        {
			HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &CAN1_Manage_Object.Rx_Buffer.Header, CAN1_Manage_Object.Rx_Buffer.Data);
			CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
		}
    }
    else if (hcan->Instance == CAN2)
    {
		// 当CAN接收FIFO1中有数据时
		while(HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) > 0)
        {
			HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &CAN2_Manage_Object.Rx_Buffer.Header, CAN2_Manage_Object.Rx_Buffer.Data);
			CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
		}
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
