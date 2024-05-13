#include "dvc_pump.h"
#include "stm32f4xx_hal.h"		// GPIO 操控所需头文件
#include "stm32f4xx_hal_gpio.h"	// GPIO 操控所需头文件

dvcPump_t dvcPump_storage, dvcPump_suction;




uint8_t DvcPump_PlatformSetSwitch(void* peripheral_handle, uint16_t pin, DVC_PUMP_SWITCHES_T switches);

DVC_PUMP_RETURN_T DvcPump_SetStatus(dvcPump_t* pump, DVC_PUMP_SWITCHES_T switches)
{
	DVC_PUMP_RETURN_T ret;
	pump->status.switches = switches;
	pump->parameter.setSwitch(	pump->parameter.peripheral_handle,
								pump->parameter.pinNum,
								pump->status.switches);
	return ret = DVC_PUMP_RETURN_OK;
}

DVC_PUMP_RETURN_T DvcPump_handleInit(dvcPump_t *pump, dvcPump_parameter_t param)
{
	DVC_PUMP_RETURN_T ret;
	// 写入参数
	memcpy((uint8_t*)&pump->parameter, (uint8_t*)&param, sizeof(param));
	// 加载外设函数
	pump->parameter.setSwitch	= DvcPump_PlatformSetSwitch;
	return ret = DVC_PUMP_RETURN_OK;
}

DVC_PUMP_RETURN_T DvcPump_InitExample(void)
{
	DVC_PUMP_RETURN_T ret;
	dvcPump_parameter_t initParam;
	// 仓库气泵初始化
	initParam.peripheral_handle = GPIOE;
	initParam.pinNum			= GPIO_PIN_11;
	DvcPump_handleInit(&dvcPump_storage, initParam);
	// 吸盘气泵初始化
	initParam.peripheral_handle = GPIOE;
	initParam.pinNum			= GPIO_PIN_13;
	DvcPump_handleInit(&dvcPump_suction, initParam);
	// 试着启动
	DvcPump_SetStatus(&dvcPump_storage, DVC_PUMP_OFF);
	DvcPump_SetStatus(&dvcPump_suction, DVC_PUMP_OFF);
	
	return ret = DVC_PUMP_RETURN_OK;
}



uint8_t DvcPump_PlatformSetSwitch(	void* peripheral_handle,
									uint16_t pin,
									DVC_PUMP_SWITCHES_T switches)
{
	uint32_t ret = DVC_PUMP_RETURN_OK;
	#if defined(MOSASAURUS_ELITE_BOARD) | defined(ROBOMASTER_DEVELOPMENT_BOARD_TYPE_C) | defined (MOSASAURUS_STEERING_CONTROL_BOARD)
		// 为了符合 HAL 库的函数，要给外设句柄和 Pin 号套一层壳再赋值
		GPIO_TypeDef* peripheral_handleShell = peripheral_handle;
		switch(switches)
		{
			case DVC_PUMP_ON:
				HAL_GPIO_WritePin(peripheral_handleShell, pin, GPIO_PIN_SET);
				break;
			case DVC_PUMP_OFF:
				HAL_GPIO_WritePin(peripheral_handleShell, pin, GPIO_PIN_RESET);
				break;
			default:
				return ret = DVC_PUMP_RETURN_ERROR;
				break;
		}
	#endif
	return ret;
}
