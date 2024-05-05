/**
 * @file drv_can.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的CAN通信初始化与配置流程
 * @version 0.1
 * @date 2022-08-02
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef DRV_CAN_H
#define DRV_CAN_H

/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

#include "stdlib.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "can.h"
/* Exported macros -----------------------------------------------------------*/

// 滤波器编号
#define CAN_FILTER(x) ((x) << 3)

// 接收队列
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

//标准帧或扩展帧
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

// 数据帧或遥控帧
#define CAN_DATA_TYPE (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)
/* Exported types ------------------------------------------------------------*/



/**
 * @brief CAN接收的信息结构体
 *
 */
typedef struct 
{
    CAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
}Struct_CAN_Rx_Buffer;

/**
 * @brief CAN通信接收回调函数数据类型
 *
 */
typedef void (*CAN_Call_Back)(Struct_CAN_Rx_Buffer *);

/**
 * @brief CAN通信处理结构体
 *
 */
typedef struct 
{
    CAN_HandleTypeDef *CAN_Handler;
    Struct_CAN_Rx_Buffer Rx_Buffer;
    CAN_Call_Back Callback_Function;
}Struct_CAN_Manage_Object;

/* Exported variables ---------------------------------------------------------*/

extern bool init_finished;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern Struct_CAN_Manage_Object CAN1_Manage_Object;
extern Struct_CAN_Manage_Object CAN2_Manage_Object;

extern uint8_t CAN1_0x1ff_Tx_Data[8];
extern uint8_t CAN1_0x200_Tx_Data[8];
extern uint8_t CAN1_0x2ff_Tx_Data[8];
extern uint8_t CAN1_0xx01_Tx_Data[8];
extern uint8_t CAN1_0xx02_Tx_Data[8];
extern uint8_t CAN1_0xx03_Tx_Data[8];
extern uint8_t CAN1_0xx04_Tx_Data[8];
extern uint8_t CAN1_0xx05_Tx_Data[8];
extern uint8_t CAN1_0xx06_Tx_Data[8];
extern uint8_t CAN1_0xx07_Tx_Data[8];
extern uint8_t CAN1_0xx08_Tx_Data[8];
extern uint8_t CAN1_0xx11_Tx_Data[8];
extern uint8_t CAN1_0xx12_Tx_Data[8];
extern uint8_t CAN1_0xx13_Tx_Data[8];
extern uint8_t CAN1_0xx14_Tx_Data[8];
extern uint8_t CAN1_0xx15_Tx_Data[8];
extern uint8_t CAN1_0xx16_Tx_Data[8];

extern uint8_t CAN2_0x1ff_Tx_Data[8];
extern uint8_t CAN2_0x200_Tx_Data[8];
extern uint8_t CAN2_0x2ff_Tx_Data[8];
extern uint8_t CAN2_0xx01_Tx_Data[8];
extern uint8_t CAN2_0xx02_Tx_Data[8];
extern uint8_t CAN2_0xx03_Tx_Data[8];
extern uint8_t CAN2_0xx04_Tx_Data[8];
extern uint8_t CAN2_0xx05_Tx_Data[8];
extern uint8_t CAN2_0xx06_Tx_Data[8];
extern uint8_t CAN2_0xx07_Tx_Data[8];
extern uint8_t CAN2_0xx08_Tx_Data[8];
extern uint8_t CAN2_0xx11_Tx_Data[8];
extern uint8_t CAN2_0xx12_Tx_Data[8];
extern uint8_t CAN2_0xx13_Tx_Data[8];
extern uint8_t CAN2_0xx14_Tx_Data[8];
extern uint8_t CAN2_0xx15_Tx_Data[8];
extern uint8_t CAN2_0xx16_Tx_Data[8];

extern uint8_t CAN_Supercap_Tx_Data[];

extern uint8_t CAN1_0xxf1_Tx_Data[8];
extern uint8_t CAN1_0xxf2_Tx_Data[8];
extern uint8_t CAN1_0xxf3_Tx_Data[8];
extern uint8_t CAN1_0xxf4_Tx_Data[8];
extern uint8_t CAN1_0xxf5_Tx_Data[8];
extern uint8_t CAN1_0xxf6_Tx_Data[8];
extern uint8_t CAN1_0xxf7_Tx_Data[8];
extern uint8_t CAN1_0xxf8_Tx_Data[8];

extern uint8_t CAN2_0xxf1_Tx_Data[8];
extern uint8_t CAN2_0xxf2_Tx_Data[8];
extern uint8_t CAN2_0xxf3_Tx_Data[8];
extern uint8_t CAN2_0xxf4_Tx_Data[8];
extern uint8_t CAN2_0xxf5_Tx_Data[8];
extern uint8_t CAN2_0xxf6_Tx_Data[8];
extern uint8_t CAN2_0xxf7_Tx_Data[8];
extern uint8_t CAN2_0xxf8_Tx_Data[8];

/* Exported function declarations ---------------------------------------------*/

void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function);

uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length, uint32_t __IDE);

void Task_CAN_PeriodElapsedCallback();
#ifdef __cplusplus
}
#endif


#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
