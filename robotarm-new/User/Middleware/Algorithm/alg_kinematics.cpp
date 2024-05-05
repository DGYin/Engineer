/* Includes ------------------------------------------------------------------*/
#include "alg_kinematics.hpp"


modifiedDh dh;



inline float AlgKine_platformSine(float input);
inline float AlgKine_platformCosine(float input);


/**
 * @brief 初始化 D-H 模型法
 *
 * @param __jointNum
 */
void modifiedDh::Init(uint8_t __jointNum)
{
	memset(this, 0, sizeof(modifiedDh));
	// 配置初始节点位置，以及其转换矩阵
	positionNPose_t base = {
	1,	0,	0,	0,
	0,	1,	0,	0,
	0,	0,	1,	0,
	0,	0,	0,	1};
	transMatrices_t baseTrans = {
	1,	0,	0,	0,
	0,	1,	0,	0,
	0,	0,	1,	0,
	0,	0,	0,	1};
	memcpy(&basePositionNPose, &base, sizeof(float)*16);
	memcpy(&jointTransMat, &baseTrans, sizeof(float)*16);
	// 写入节点数量
	jointNum = __jointNum;
	Dh_param_t initDh;
	// 关节0 D-H 参数设置
	
	// D-H 法过程
	// 先沿 X 轴转动转轴，使之能沿新坐标系的 X 轴到达过度点 （对应alpha）
	// 再沿 X 轴移动到过度点 （对应a）
	// 再将过度点坐标沿 Z 轴旋转，使姿态角一致 （对应theta）
	// 再沿 Z 轴移动到重点 （对应d）
	initDh.jointType = DH_JOINT_TYPE_PRISMATIC;
	initDh.theta	= 0;
	initDh.d		= 0;
	initDh.a		= 0;
	initDh.alpha	= 0;
	initDh.sourceOffset = 0;
	SetJointDhParam(0, initDh);
	// 关节1 D-H 参数设置
	initDh.jointType = DH_JOINT_TYPE_REVOLUTE;
	initDh.theta	= 0;
	initDh.d		= 0;
	initDh.a		= 0;
	initDh.alpha	= 0;
	initDh.sourceOffset = 0;
	SetJointDhParam(1, initDh);
	// 关节2 D-H 参数设置
	initDh.theta	= 0;
	initDh.d		= 0;
	initDh.a		= 3;
	initDh.alpha	= 0;
	initDh.sourceOffset = 0;
	SetJointDhParam(2, initDh);
	// 关节3 D-H 参数设置
	initDh.theta	= 0;
	initDh.d		= 0;
	initDh.a		= 3;
	initDh.alpha	= 0;
	initDh.sourceOffset = 0;
	SetJointDhParam(3, initDh);
	// 关节4 D-H 参数设置
	initDh.theta	= 0;
	initDh.d		= 0;
	initDh.a		= 1;
	initDh.alpha	= PI/2.f;
	initDh.sourceOffset	= -PI/2.f;
	SetJointDhParam(4, initDh);
	// 关节5 D-H 参数设置
	initDh.theta	= 0;
	initDh.d		= 0.5;
	initDh.a		= 0;
	initDh.alpha	= -PI/2.f;
	initDh.sourceOffset = 0;
	SetJointDhParam(5, initDh);
	// 进行一次正运动学解算
	FKine();
}

/**
 * @brief 设置每个节点的 D-H 模型法参数
 *
 * @param id 节点的 ID
 * @param dh D-H 模型法的参数
 * @return DH_RETURN_T
 */
DH_RETURN_T modifiedDh::SetJointDhParam(uint8_t id, Dh_param_t dh)
{
	// 判断 ID 有效性
	if (id >= MAXIMUM_JOINT_NUM) return DH_RETURN_ERROR;
	// 输入参数
	memcpy(&jointDhParamList[id], &dh, sizeof(Dh_param_t));
	return DH_RETURN_OK;
}

/**
 * @brief 进行一次正向运动学解算
 *
 * @return DH_RETURN_T
 */
DH_RETURN_T modifiedDh::FKine(void)
{
	transMatrices_t tempMat;
	// 关节0为基点
	memcpy(&jointPositionNPose[0], &basePositionNPose, sizeof(basePositionNPose));
	
	// 再逐个更新后续关节
	for (int i=0; i<jointNum; i++)
	{
		// 获得一个转换矩阵
		GetTransMatrices(jointDhParamList[i], &tempMat);
		// 进行连乘，得到当前 frame 相对基点的转换矩阵进行存储
		arm_matrix_instance_f32 historyMat, currentMat, resultMat;
		arm_mat_init_f32(&historyMat,	4, 4, (float*)jointTransMat[i]);
		arm_mat_init_f32(&currentMat,	4, 4, (float*)tempMat);
		arm_mat_init_f32(&resultMat,	4, 4, (float*)jointTransMat[i+1]);
		arm_mat_mult_f32(&historyMat, &currentMat, &resultMat); // Tn = T1 . T2 ... Tn
		// 使用转换矩阵获取当前 Frame 的姿态
		arm_matrix_instance_f32 transMat, basePosMat, framePosMat;
		arm_mat_init_f32(&transMat,		4, 4, (float*)jointTransMat[i]);
		arm_mat_init_f32(&basePosMat,	4, 4, (float*)basePositionNPose);
		arm_mat_init_f32(&framePosMat,	4, 4, (float*)jointPositionNPose[i]);
		arm_mat_mult_f32(&transMat, &basePosMat, &framePosMat); // Pn = Tn dot Pbase
		//SingleJointForwardKinematics(&jointPositionNPose[i], &jointPositionNPose[i-1], jointDhParamList[i-1]); // 根据上一个的状态推断当前的关节
	}
	return DH_RETURN_OK;
}




DH_RETURN_T modifiedDh::GetTransMatrices(Dh_param_t dh, transMatrices_t* tMat)
{
	// 计算转换矩阵所需内容
	float st, ct;
	float sa = AlgKine_platformSine(dh.alpha);
	float ca = AlgKine_platformCosine(dh.alpha);
	float d, q;
	//if (dh.source == NULL) return DH_RETURN_ERROR; // 防止空指针报错
	memcpy(&q, dh.source, sizeof(float));
	q = 0;
	q = q + dh.sourceOffset;
	switch ((uint8_t) dh.jointType)
	{
		case DH_JOINT_TYPE_REVOLUTE:
			st	= AlgKine_platformSine(q);
			ct	= AlgKine_platformCosine(q);
			d	= dh.d;
			break;
		case DH_JOINT_TYPE_PRISMATIC:
			st	= AlgKine_platformSine(dh.theta);
			ct	= AlgKine_platformCosine(dh.theta);
			d	= q;
			break;
		default:
			return DH_RETURN_ERROR;
			break;
	}
	// 改进法的转换矩阵计算公式
	float transMat[16] = {
		ct,		-st,	0,		dh.a,
		st*ca,	ct*ca,	-sa,	-sa*d,
		st*sa,	ct*sa,	ca,		ca*d,
		0,		0,       0,		1};
	// 装填数据
	memcpy(tMat, &transMat, sizeof(float)*16);
	return DH_RETURN_OK;
}

/**
 * @brief 自定义 Sine 函数
 *
 * @param input
 * @return float
 */
inline float AlgKine_platformSine(float input)
{
	return arm_sin_f32(input);
}

/**
 * @brief 自定义 Cosine 函数
 *
 * @param input
 * @return float
 */
inline float AlgKine_platformCosine(float input)
{
	return arm_cos_f32(input);
}
