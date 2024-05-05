/**
 * @file crt_gimbal.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 云台电控
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "controller_task.h"

/* Private macros ------------------------------------------------------------*/
extern osSemaphoreId AngleSlover_SemHandle;
/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

void Class_Controller::Init()
{
	Opticalflow.Init(&huart1);
	Referee.Init(&huart2);
}

void Class_Controller::ToEulerAngles()
{
	osSemaphoreWait(AngleSlover_SemHandle,osWaitForever);
	float q[4] = {0.0f};
	Struct_Quaternion_Data Quaternion_Data = Opticalflow.Get_Quaternion_Data();
	for(uint8_t i=0;i<4;i++)
	{
		q[i] = (float)(Quaternion_Data.V[i])/10000.0f;
	}
	// roll (x-axis rotation)
    float sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
    double cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
    EulerAngles.roll = atan2(sinr_cosp, cosr_cosp)*RAD_TO_DEG;
 
    // pitch (y-axis rotation)
    double sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
    if (abs(sinp) >= 1)
        EulerAngles.pitch = copysign(PI / 2, sinp)*RAD_TO_DEG; // use 90 degrees if out of range
    else
        EulerAngles.pitch = asin(sinp)*RAD_TO_DEG;
 
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
    double cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
    EulerAngles.yaw = atan2(siny_cosp, cosy_cosp)*RAD_TO_DEG;
}

float Delta_Yaw = 0.0f;
void Class_Controller::Communication_To_Robot()
{
	Struct_OpticalFlow_Integrate_Data_2 temp_OpticalFlow = Opticalflow.Get_OpticalFlow_Integrate_Data_2();
	
	static float Last_Yaw = 0.0f;
	static uint8_t count = 0;
	if((temp_OpticalFlow.Quality >=150) && (temp_OpticalFlow.State == 1) && (Opticalflow.Get_OpticalFlow_Status() == OpticalFlow_Status_ENABLE))
	{
		Math_Constrain(temp_OpticalFlow.DX_FIX,(int16_t)-40,(int16_t)40);
		Math_Constrain(temp_OpticalFlow.DY_FIX,(int16_t)-40,(int16_t)40);
	}
	else
	{
		temp_OpticalFlow.DX_FIX = 0;
		temp_OpticalFlow.DY_FIX = 0;
	}
	
	if(fabs(EulerAngles.yaw - Last_Yaw) <= 0.1f)
	{
		count++;
		if(count >= 60)
		{
			Offset_Yaw = EulerAngles.yaw;
			count = 0;
		}
	}
	Last_Yaw = EulerAngles.yaw;
	Delta_Yaw= EulerAngles.yaw - Offset_Yaw;
	Communication_Data.Flow_x = Math_Float_To_Int((float)temp_OpticalFlow.DX_FIX/40.0f , -1.0f , 1.0f , 0 , (1 << 16) - 1);
	Communication_Data.Flow_y = Math_Float_To_Int((float)temp_OpticalFlow.DY_FIX/40.0f , -1.0f , 1.0f , 0 , (1 << 16) - 1);
	Communication_Data.pitch = Math_Float_To_Int(EulerAngles.pitch/90.0f , -1.0f , 1.0f , 0 ,(1 << 16) - 1);
	Communication_Data.roll = Math_Float_To_Int(EulerAngles.roll/90.0f , -1.0f , 1.0f , 0 ,(1 << 16) - 1);
	Communication_Data.yaw = Math_Float_To_Int(EulerAngles.yaw/180.0f , -1.0f , 1.0f , 0 ,(1 << 16) - 1);
//	memset(&EulerAngles,0,sizeof(Struct_EulerAngles));
	memcpy(&Referee.Interaction_Custom_Controller,&Communication_Data,sizeof(Communication_Data));
	Referee.Data_Concatenation(Referee_Command_ID_INTERACTION_CUSTOM_CONTROLLER,(uint8_t*)&Referee.Interaction_Custom_Controller,sizeof(Referee.Interaction_Custom_Controller)-2);
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
