/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ALG_KINEMATICS_H
#define ALG_KINEMATICS_H

//#ifdef __cplusplus
//extern "C" {
//#endif

	
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include "arm_math.h"
	
#define MAXIMUM_JOINT_NUM 8
#define PI 3.1415
#define DEGREE_TO_RAD 0.01745
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

typedef float positionNPose_t[16];
typedef float transMatrices_t[16];
typedef enum
{
	DH_JOINT_TYPE_REVOLUTE	= 0x00U,
	DH_JOINT_TYPE_PRISMATIC	= 0x01U,
} DH_JOINT_TYPE_T;

typedef enum
{
	DH_RETURN_OK,
	DH_RETURN_ERROR,
	
} 	DH_RETURN_T;
	
typedef struct
{
	DH_JOINT_TYPE_T jointType;
	float	theta;	// link angle	关节转角
	float	d;		// link offset	关节距离
	float	alpha;	// link twist	连杆扭角
	float	a;		// link length	连杆长度
	float	sourceOffset; //offset	偏移		
	float*	source; // 数据源（如电机角度的弧度值、导轨高度）
	float	sourceMaximum;
	float	sourceMinimum;
} Dh_param_t;


class modifiedDh
{
	public:
		// 用于初始化列表
		void Init(uint8_t jointNum);
		// 用于初始化节点
		DH_RETURN_T SetJointDhParam(uint8_t id, Dh_param_t Dh);
		// 运动学解算
		DH_RETURN_T FKine(void);	// 正运动学解算
		DH_RETURN_T IKine(void);	// 逆运动学解算（数值法）
		// 进行一次正向运动学解算
	protected:
		uint8_t jointNum;
		
		
		// 设置起始位置
		positionNPose_t basePositionNPose;
		// 运动学核心成员
		Dh_param_t jointDhParamList[MAXIMUM_JOINT_NUM];			// 关节的改进 D-H 法参数
		positionNPose_t jointPositionNPose[MAXIMUM_JOINT_NUM];	// 存储每个关节相对起始位置的位姿
		transMatrices_t jointTransMat[MAXIMUM_JOINT_NUM];		// 存储当前关节相对于起始位姿的转换矩阵
		DH_RETURN_T GetTransMatrices(Dh_param_t Dh, transMatrices_t* tMat);	// 得到单个转换矩阵
};

extern modifiedDh dh;

//#ifdef __cplusplus
//}
//#endif

#endif
