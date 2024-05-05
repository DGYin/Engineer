#include "alg_smoothen.h"

algSmoothen_uniformSmoothen_t joint1AngleSmoothen, joint2AngleSmoothen;

ALG_SMOOTHEN_RETURN_T AlgSmoothen_UniformSmoothenCore(algSmoothen_uniformSmoothen_t* ush);

/**
 * @brief 用于将输出进行平缓化
 *
 * @param ush 句柄
 * @param input 当前值
 * @param target 目标值
 * @param output 输出值
 * @return ALG_SMOOTHEN_RETURN_T
 */
ALG_SMOOTHEN_RETURN_T AlgSmoothen_UniformSmoothen(	algSmoothen_uniformSmoothen_t* ush,
													float input, float target, float* output)
{
	uint32_t ret = ALG_SMOOTHEN_OK;
	ush->input	= input;
	ush->target	= target;
	ret &= AlgSmoothen_UniformSmoothenCore(ush);	// 运算函数
	*output	= ush->output;
	return ret;

}

/**
 * @brief 用于设置恒速度平缓化的变化量
 *
 * @param ush 句柄
 * @param posDel 正速度
 * @param negDel 负速度
 * @return ALG_SMOOTHEN_RETURN_T
 */
ALG_SMOOTHEN_RETURN_T AlgSmoothen_SetUsDelta(	algSmoothen_uniformSmoothen_t* ush,
												float posDel, float negDel)
{
	uint32_t ret = ALG_SMOOTHEN_OK;
	ush->positiveDelta	= posDel;
	ush->negativeDelta	= negDel;
	return ret;
}

ALG_SMOOTHEN_RETURN_T AlgSmoothen_SetUsType(	algSmoothen_uniformSmoothen_t* ush,
												ALG_SMOOTHEN_UNIFORM_SMOOTHEN_TYPE_T type)
{
	uint32_t ret = ALG_SMOOTHEN_OK;
	ush->uniformType	= type;
	return ret;
}

/**
 * @brief 恒速度平缓化的核心代码
 *
 * @param ush 句柄
 * @return ALG_SMOOTHEN_RETURN_T
 */
ALG_SMOOTHEN_RETURN_T AlgSmoothen_UniformSmoothenCore(algSmoothen_uniformSmoothen_t* ush)
{
	float diff;
	diff = ush->target - ush->input;	// 计算差值
	switch ((uint8_t) ush->uniformType)
	{
		case UNIFORM_SMOOTHEN_TYPE_ABSOLUTE:
			// 最常见的斜波函数，懒得写了。Return 个 error 意思意思。
			return ALG_SMOOTHEN_ERROR;
			break;
		case UNIFORM_SMOOTHEN_TYPE_RELATIVE:
			// 正增量部分
			if (diff>0 && diff>ush->positiveDelta)	// 目标值大于输入，且差值大于正增量
				ush->output = ush->input + ush->positiveDelta;	// 增量计算
			else if (diff>0)
				ush->output = ush->target;	// 需要增量，但插值小于目标值，直接等于目标值
			// 负增量部分
			if (diff<0 && -diff>ush->negativeDelta)	// 目标值小于输入，且差值大于负增量
				ush->output = ush->input - ush->negativeDelta;	// 增量计算
			else if (diff<0)
				ush->output = ush->target;	// 需要减量，但差值小于目标值，直接等于目标值
			break;
		default:
			return ALG_SMOOTHEN_ERROR;
			break;
	}
	return ALG_SMOOTHEN_OK;
}

/**
 * @brief 恒速度平缓化句柄初始化示例
 *
 * @return ALG_SMOOTHEN_RETURN_T
 */
ALG_SMOOTHEN_RETURN_T AlgSmoothen_UsHanldeInitExample(	algSmoothen_uniformSmoothen_t* ush,
														ALG_SMOOTHEN_UNIFORM_SMOOTHEN_TYPE_T usType)
{
	// 关节电机1 角度平滑句柄初始化
	ush->uniformType	= usType;
	AlgSmoothen_SetUsDelta(ush, 1, 1);

	return ALG_SMOOTHEN_OK;
};
