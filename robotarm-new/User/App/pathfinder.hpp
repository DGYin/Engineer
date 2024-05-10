#ifndef PATHFINDER_HPP
#define PATHFINDER_HPP


#include "stdint.h"
#include "math.h"
#include "string.h"


typedef enum{
	PATHFINDER_OK		= 0x00U,
	PATHFINDER_ERROR	= 0x01U,
} PATHFINDER_RETURN_T;

typedef enum{
	PATHFINDER_ON		= 0x00U,
	PATHFINDER_OFF		= 0x01U,
} PATHFINDER_SWITCH_T;

typedef enum{
	PATHFINDER_UNREADY		= 0x00U,
	PATHFINDER_READY		= 0x01U,
} PATHFINDER_READY_CODE_T;

typedef enum{
	PATHFINDER_FORCE_STOP_PRIORITY		= 0x00U,
	PATHFINDER_GO_TO_DEFAULT_PRIORITY	= 0x05U,
	PATHFINDER_SILVER_ORE_PICK_PRIORITY	= 0x0AU,
	PATHFINDER_FREE_PRIORITY			= 0xFFU,
} PATHFINDER_TASK_TO_PRIORITY_LUT_T;

typedef float qMatrix_t[6];			// 用于返回打点记录的控制量 Q 矩阵

class pathfinder_c{
	public:
		// 控制相关
		PATHFINDER_RETURN_T Pathfinder_Init();
		PATHFINDER_RETURN_T Pathfinder_TaskScheduler();
		PATHFINDER_RETURN_T Pathfinder_SetTask(PATHFINDER_TASK_TO_PRIORITY_LUT_T task);
		PATHFINDER_RETURN_T Pathfinder_CheckReady(float* pInput);
		// 输出相关
		PATHFINDER_RETURN_T Pathfinder_GetQMat(qMatrix_t qMat);
	private:
		// 任务核心函数
		PATHFINDER_RETURN_T Pathfinder_DoSilverOrePick();
		// 状态成员
		struct{
			uint32_t pathfinderTick;
			PATHFINDER_SWITCH_T switching;
			PATHFINDER_READY_CODE_T readyCode;
			PATHFINDER_TASK_TO_PRIORITY_LUT_T lastTask;
			PATHFINDER_TASK_TO_PRIORITY_LUT_T currentTask;
		} status;
		// 用于返回的 Q 成员
		qMatrix_t qMatOutput;
		// 用于判断是否准备好执行路径任务
		float readyToleranceRad		= 0.2;
		float readyToleranceMeter	= 0.02;
		PATHFINDER_RETURN_T Pathfinder_CheckTolerance(float input, float target, float tole);
};

extern pathfinder_c pathfinder;
#endif

