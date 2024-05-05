/*
	Independent Commission Against Cracking, 缩写 ICAC
	用来监视某个组件是否离线
	
	By Jason Yin
	
*/



/* Includes ---------------------------------------------------------------------*/
#include "icac.h"
#include "DM_motor.h"
#include "AK_motor.h"


icac_t icac_dmMotor_joint3, icac_akMotor_joint1, icac_akMotor_joint2;


uint16_t icacListedNum = 0;
icacCallingListElement_t icacCallingList[ICAC_LIST_MAXIMUM_NUM];

void Icac_TaskScheduler(void)
{
	for (int i=0; i<icacListedNum; i++) // 遍历所有需要监视的句柄
	{
		icacCallingList[i].remainningMs--; // 计时处理
		if (icacCallingList[i].remainningMs == 0) // 到达调度时长
		{
			icacCallingList[i].remainningMs = icacCallingList[i].icac->parameter.callingPeriodMs; // 重置计时
			switch(icacCallingList[i].icac->parameter.icacType) // 根据不同类型选择不同策略
			{
				case ICAC_TYPE_COUNTER:	// 计数器类型
					
					memcpy(	(uint8_t*)&icacCallingList[i].icac->feedback.nowCounter,
							(uint8_t*)icacCallingList[i].icac->parameter.pCounter,
							sizeof(icacCallingList[i].icac->feedback.nowCounter)); // 从设定的计数器位置拿出数据
					if (icacCallingList[i].icac->feedback.nowCounter > icacCallingList[i].icac->feedback.lastCounter){ // 与上次比较，计数器仍在增加
						icacCallingList[i].icac->status.connectionStatus = ICAC_ONLINE;
					}
					else{
						icacCallingList[i].icac->status.connectionStatus = ICAC_OFFLINE;
					}
					icacCallingList[i].icac->feedback.lastCounter = icacCallingList[i].icac->feedback.nowCounter; // 记录上次数据
					break;
				case ICAC_TYPE_WATCHDOG: // 看门狗类型，还没写
					break;
				case ICAC_TYPE_KEEPALIVE: // 探测保活类型，还没写
					break;
				default:
					return;
					break;
			}
			
		}
	}
}

ICAC_RETURN_T Icac_CounterInit(icac_t* icac, uint32_t callingPeriodMs, uint32_t* pCounter)
{
	uint32_t ret;
	icac->parameter.icacType		= ICAC_TYPE_COUNTER;
	icac->parameter.callingPeriodMs	= callingPeriodMs;
	icac->parameter.pCounter		= pCounter;
	if (icacListedNum < ICAC_LIST_MAXIMUM_NUM)
	{
		icac->parameter.icacListPosition	= icacListedNum++;
		icacCallingList[icac->parameter.icacListPosition].icac 			= icac;
		icacCallingList[icac->parameter.icacListPosition].remainningMs	= callingPeriodMs;
	}
	else
		return ret = ICAC_ERROR;
	
}

ICAC_CONNECTION_STATUS_T Icac_GetConnectionStatus(icac_t* icac)
{
	ICAC_CONNECTION_STATUS_T ret;
	return ret = icac->status.connectionStatus;
}

void Icac_HandleInit(void)
{
	
	Icac_CounterInit(&icac_akMotor_joint1, 200, &dmMotor_joint4.status.resolvedCnt);
	Icac_CounterInit(&icac_akMotor_joint2, 200, &dmMotor_joint4.status.resolvedCnt);
	Icac_CounterInit(&icac_dmMotor_joint3, 200, &dmMotor_joint4.status.resolvedCnt);
}