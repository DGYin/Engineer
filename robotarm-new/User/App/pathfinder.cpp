#include "pathfinder.hpp"
#include "pathfinderLut.h"
pathfinder_c pathfinder;

PATHFINDER_RETURN_T pathfinder_c::Pathfinder_Init()
{
	PATHFINDER_RETURN_T ret;
	status.currentTask	= PATHFINDER_FREE_PRIORITY;	// 默认为空闲
	status.switching	= PATHFINDER_OFF;			// 默认为无任务状态
	return ret = PATHFINDER_OK;
}

PATHFINDER_RETURN_T pathfinder_c::Pathfinder_TaskScheduler()
{
	PATHFINDER_RETURN_T ret;
	if (status.currentTask!=status.lastTask && status.currentTask!=PATHFINDER_FREE_PRIORITY)	// 有新的路径任务，tick归零
	{
		status.pathfinderTick = 0;
		status.switching = PATHFINDER_ON;
	}
			
	status.lastTask = status.currentTask;
	if (status.switching==PATHFINDER_ON && status.readyCode==PATHFINDER_READY)	// 如果当前正处于路径任务执行状态，且准备好开始了，则计时 Tick + 1
	{
		status.pathfinderTick++;
		switch (status.currentTask)						// 根据当前任务，选择不同方案
		{
			case PATHFINDER_FORCE_STOP_PRIORITY:		// 强制停止路径任务
				status.switching = PATHFINDER_OFF;
				return ret = PATHFINDER_OK;
				break;
			case PATHFINDER_GO_TO_DEFAULT_PRIORITY:		// 回到默认位置任务
				return ret = PATHFINDER_OK;
			case PATHFINDER_SILVER_ORE_PICK_PRIORITY:	// 银矿拾取任务
				Pathfinder_DoSilverOrePick();
				return ret = PATHFINDER_OK;
			case PATHFINDER_FREE_PRIORITY:				// 空闲状态
				return ret = PATHFINDER_OK;
			default:
				return ret = PATHFINDER_ERROR;			// 其他状态，返回错误
		}
	}
	else
		return ret = PATHFINDER_ERROR;
}

PATHFINDER_RETURN_T pathfinder_c::Pathfinder_SetTask(PATHFINDER_TASK_TO_PRIORITY_LUT_T task)
{
	PATHFINDER_RETURN_T ret;
	if (task < status.currentTask)	// 高优先级抢占成果
	{
		status.switching	= PATHFINDER_ON;
		status.currentTask	= task;
		return ret = PATHFINDER_OK;
	}
	return ret = PATHFINDER_ERROR;	// 抢占失败，返回 error
}

PATHFINDER_RETURN_T pathfinder_c::Pathfinder_CheckReady(float* pInput)
{
	PATHFINDER_RETURN_T ret;
	uint8_t check = 0;
	if (status.readyCode == PATHFINDER_UNREADY)	// 没准备好才需要检查
	{
		switch(status.currentTask)	// 根据不同任务策略进行不同 Check 方案
		{
			case PATHFINDER_SILVER_ORE_PICK_PRIORITY:	// 银矿拾取任务
				check |= Pathfinder_CheckTolerance(pInput[0], pathfinder_silverOrePick_tick2QMatLut[0], readyToleranceMeter);
				check |= Pathfinder_CheckTolerance(pInput[1], pathfinder_silverOrePick_tick2QMatLut[1], readyToleranceRad);
				check |= Pathfinder_CheckTolerance(pInput[2], pathfinder_silverOrePick_tick2QMatLut[2], readyToleranceRad);
				check |= Pathfinder_CheckTolerance(pInput[3], pathfinder_silverOrePick_tick2QMatLut[3], readyToleranceRad);
				if (check) status.readyCode = PATHFINDER_UNREADY;	// 利用取或，任意为 Error 就返回 1
					else status.readyCode = PATHFINDER_READY;		// 仍为0， 则没有返回 Error，转换为 Ready
				return ret = PATHFINDER_OK;
				break;
			case PATHFINDER_FREE_PRIORITY:				// 空闲任务，没必要检查准备好没有，返回错误
				status.readyCode = PATHFINDER_UNREADY;
				return ret = PATHFINDER_ERROR;
				break;
			default:									// 错误的任务码，返回错误
				status.readyCode = PATHFINDER_UNREADY;
				return ret = PATHFINDER_ERROR;
				break;
		}
	}
	else return ret = PATHFINDER_ERROR;
}


PATHFINDER_RETURN_T pathfinder_c::Pathfinder_GetQMat(qMatrix_t qMat)
{
	PATHFINDER_RETURN_T ret;
	if (status.readyCode == PATHFINDER_READY)	// 准备好了，就开始输出序列目标值
	{
		memcpy(qMat, qMatOutput, 6*sizeof(float));
		return ret = PATHFINDER_OK;
	}
	if (status.readyCode == PATHFINDER_UNREADY)	// 没准备好，就返回对应任务的初始值
	{
		switch(status.currentTask)	// 根据任务码，返回初始值
		{
			case PATHFINDER_SILVER_ORE_PICK_PRIORITY:	// 银矿拾取任务
				for(int i=0; i<4; i++)
					qMat[i] = pathfinder_silverOrePick_tick2QMatLut[i];
				qMatOutput[4] = 0;
				qMatOutput[5] = 0;
				return ret = PATHFINDER_OK;
				break;
			default:									// 没有任务，返回错误
				return ret = PATHFINDER_ERROR;
				break;
		}
	}
	return ret = PATHFINDER_ERROR;
}

PATHFINDER_RETURN_T pathfinder_c::Pathfinder_DoSilverOrePick()
{
	PATHFINDER_RETURN_T ret;
	uint8_t lutIndex = 0;
	uint32_t tickNow = status.pathfinderTick;
	uint32_t tickStart;
	uint32_t tickEnd;
	uint32_t tickWholeProcess;
	// 算出全流程时长
	for (int i=0; i<pf_silverOrePick_IndexNum; i++)
		tickWholeProcess = tickWholeProcess+pathfinder_silverOrePick_tick2IndexLut[i];
	// 处于流程时长内
	if (tickNow < tickWholeProcess)
	{
		// 查表
		for (int index=0; index<pf_silverOrePick_IndexNum; index++)
		{
			// 算出每个动作的起止时间
			if (index==0) 
			{
				tickStart	= 0;
				tickEnd		= pathfinder_silverOrePick_tick2IndexLut[1];
			}
			else{
				tickStart	= tickStart+pathfinder_silverOrePick_tick2IndexLut[index-1];
				tickEnd		= tickStart+pathfinder_silverOrePick_tick2IndexLut[index];
			}
			// 处于某个动作的时间范围内，执行动作
			if (tickNow>tickStart && tickNow<tickEnd)
			{
				for (int i=0; i<4; i++)	// 查表得到输出qMat
					qMatOutput[i] = pathfinder_silverOrePick_tick2QMatLut[index*4 + i];
				qMatOutput[4] = 0;
				qMatOutput[5] = 0;
				break;	// 退出查表
			}
		}
	}
	else
	{
		status.readyCode	= PATHFINDER_UNREADY;		// 标记为没准备好状态
		status.currentTask	= PATHFINDER_FREE_PRIORITY;	// 归还优先级
		status.switching	= PATHFINDER_OFF;			// 关闭寻路
		status.pathfinderTick = 0;						// 归还计时器
	}
	return ret = PATHFINDER_OK;
}

PATHFINDER_RETURN_T pathfinder_c::Pathfinder_CheckTolerance(float input, float target, float tole)
{
	PATHFINDER_RETURN_T ret;
	if (fabs(input - target) < tole)
		return ret = PATHFINDER_OK;
	return ret = PATHFINDER_ERROR;
};