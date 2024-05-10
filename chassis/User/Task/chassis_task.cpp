/**
 * @file crt_chassis.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 底盘电控
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/**
 * @brief 轮组编号
 * 1 2
 * 3 4
 */

/* Includes ------------------------------------------------------------------*/

#include "chassis_task.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Class_Chassis Chassis;
/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 底盘初始化
 *
 * @param __Chassis_Control_Type 底盘控制方式, 默认麦轮方式
 * @param __Speed 底盘速度限制最大值
 */
void Class_Chassis::Init(float __Velocity_X_Max, float __Velocity_Y_Max, float __Omega_Max)
{
    Velocity_X_Max = __Velocity_X_Max;
    Velocity_Y_Max = __Velocity_Y_Max;
    Omega_Max = __Omega_Max;
	//裁判系统初始化
	Referee.Init(&huart1);
	Roboatrm_Communication.Init(&hcan2);
    //斜坡函数加减速速度X
    Slope_Velocity_X.Init(0.0005f, 0.01f);
    //斜坡函数加减速速度Y
    Slope_Velocity_Y.Init(0.0005f, 0.01f);
    //斜坡函数加减速角速度
    Slope_Omega.Init(0.0005f, 0.05f);

    //电机PID批量初始化
    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].PID_Omega.Init(1000.0f, 0.0f, 0.01f, 0.0f, Motor_Wheel[i].Get_Output_Max(), Motor_Wheel[i].Get_Output_Max());
    }
//	Motor_Rise.PID_Angle.Init(0.5f, 0.0f, 0.01f, 0.0f, 1.0f * PI, 1.0f * PI);
//	Motor_Rise.PID_Omega.Init(4000.0f, 0.1f, 0.0000f, 0, Motor_Rise.Get_Output_Max(), Motor_Rise.Get_Output_Max());
	
    //轮向电机
    Motor_Wheel[0].Init(&hcan1, DJI_Motor_ID_0x201);
    Motor_Wheel[1].Init(&hcan1, DJI_Motor_ID_0x202);
    Motor_Wheel[2].Init(&hcan1, DJI_Motor_ID_0x203);
	Motor_Wheel[3].Init(&hcan1, DJI_Motor_ID_0x204);
//	Motor_Rise.Init(&hcan1, DJI_Motor_ID_0x205);
}


//bool Class_Chassis::Rise_Calibration(float Cali_Omega,float Cali_Max_Out, float Target_Height)
//{
//	//记录电机堵转时间
//	static uint16_t count;
//	//设置为速度环校准
//	Motor_Rise.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
//	Motor_Rise.Set_Target_Omega(Cali_Omega);
//	Motor_Rise.PID_Omega.Set_Out_Max(Cali_Max_Out);
//	
//	//当电流值大于阈值，同时速度小于一定阈值，判定为堵转条件
//	if( (fabs(Motor_Rise.Get_Now_Torque()) >= 2000) && (fabs(Motor_Rise.Get_Now_Omega()) < 0.01*PI) )
//	{
//		count++;
//		//当到达一定时间，判定为堵转
//		if(count >= 200)
//		{
//			//改为角度环，设置关节角度
//			Motor_Rise.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
//			Motor_Rise.PID_Omega.Set_Out_Max(16384);
//			Rise_Offset_Angle = Motor_Rise.Get_Now_Angle();
//			Robotarm_Height = Target_Height;
//			count = 0;
//			return true;
//		}	
//	}
//	return false;
//}

/**
 * @brief 输出到电机
 *
 */
void Class_Chassis::Output()
{
    if (Chassis_Control_Type == Chassis_Control_Type_DISABLE)
    {
        //底盘失能
		for(auto& Motor:Motor_Wheel)
		{
			Motor.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
			Motor.PID_Angle.Set_Integral_Error(0.0f);
			Motor.PID_Omega.Set_Integral_Error(0.0f);
			Motor.Set_Target_Torque(0.0f);
		}
    }
    else if (Chassis_Control_Type == Chassis_Control_Type_ENABLE)
    {
		for(auto& Motor:Motor_Wheel)
		{
			Motor.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
		}
    }

//	if (Rise_Control_Type == Rise_Control_Type_DISABLE)
//	{
//		Motor_Rise.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
//		Motor_Rise.PID_Angle.Set_Integral_Error(0.0f);
//		Motor_Rise.PID_Omega.Set_Integral_Error(0.0f);
//		Motor_Rise.Set_Target_Torque(0.0f);
//	}
//	else if (Rise_Control_Type == Rise_Control_Type_ENABLE && (Motor_Rise.Get_Control_Method() != DJI_Motor_Control_Method_OMEGA))
//	{
//		Motor_Rise.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
//		Set_Motor_Angle(Motor_Rise);
//	}
	
}

/**
 * @brief 底盘逆解算
 */
void Class_Chassis::Inverse_Kinematic()
{
	float Temp_Speed[4];

	//电机转速 rpm
	Temp_Speed[0] = -((-Slope_Velocity_X.Get_Out() + Slope_Velocity_Y.Get_Out() + Slope_Omega.Get_Out() *(Half_Width + Half_Length))
						/ Wheel_Radius * RADPS_TO_RPM);
	Temp_Speed[1] = ((Slope_Velocity_X.Get_Out() + Slope_Velocity_Y.Get_Out() - Slope_Omega.Get_Out() *(Half_Width + Half_Length))
						/ Wheel_Radius * RADPS_TO_RPM);
	Temp_Speed[2] = ((-Slope_Velocity_X.Get_Out() + Slope_Velocity_Y.Get_Out() - Slope_Omega.Get_Out() *(Half_Width + Half_Length))
						/ Wheel_Radius * RADPS_TO_RPM);
	Temp_Speed[3] = -((Slope_Velocity_X.Get_Out() + Slope_Velocity_Y.Get_Out() + Slope_Omega.Get_Out() *(Half_Width + Half_Length))
						/ Wheel_Radius * RADPS_TO_RPM);
	for(uint8_t i=0;i<4;i++)
	{
		Motor_Wheel[i].Set_Target_Omega(Temp_Speed[i]);
	}
}


void Class_Chassis::Target_Resolution()
{
	Struct_Data_ID_0x11 temp_Tx_Data_ID_0x11 = Roboatrm_Communication.Get_Data_ID_0x11();
	Target_Velocity_X = Math_Int_To_Float(temp_Tx_Data_ID_0x11.Chassis_Vx, 0, ((1 << 16) - 1), -1.0f, 1.0f);
	Target_Velocity_Y = Math_Int_To_Float(temp_Tx_Data_ID_0x11.Chassis_Vy, 0, ((1 << 16) - 1), -1.0f, 1.0f);
	Target_Omega = -Math_Int_To_Float(temp_Tx_Data_ID_0x11.Chassis_Wz, 0, ((1 << 16) - 1), -1.0f, 1.0f);
	Robotarm_Height = Math_Int_To_Float(temp_Tx_Data_ID_0x11.Robotarm_Pz, 0, ((1 << 16) - 1), 0, 1160.0f);
	
	//斜坡函数计算用于速度解算初始值获取
    Slope_Velocity_X.Set_Target(Target_Velocity_X);
    Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();
    Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
    Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();
    Slope_Omega.Set_Target(Target_Omega);
    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();
}

//void Class_Chassis::Rise_Control()
//{
//	static bool Rise_Cal_Flag = false;
//	if(Rise_Cal_Flag == false)
//	{
//		if(Rise_Calibration(-0.2*PI , 4000 , 20) == true)
//		{
//			Rise_Cal_Flag = true;
//		}
//	}
//	else if(Rise_Control_Type == Rise_Control_Type_DISABLE)
//	{
//		Rise_Cal_Flag =false;
//	}
//}
/**
 * @brief 底盘状态检测回调函数
 *
 */
void Class_Chassis::Task_Alive_PeriodElapsedCallback()
{
	//底盘检测
	for(auto& Motor:Motor_Wheel)
	{
		Motor.Task_Alive_PeriodElapsedCallback();
	}
	if((Motor_Wheel[0].Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE)
	&& (Motor_Wheel[1].Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE)
	&& (Motor_Wheel[2].Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE)
	&& (Motor_Wheel[3].Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE))
	{
		Chassis_Control_Type = Chassis_Control_Type_ENABLE;
	}
	else
	{
		Chassis_Control_Type = Chassis_Control_Type_DISABLE;
	}
	//抬升电机检测
//	Motor_Rise.Task_Alive_PeriodElapsedCallback();
//	if(Motor_Rise.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE)
//	{
//		Rise_Control_Type = Rise_Control_Type_ENABLE;
//	}
//	else
//	{
//		Rise_Control_Type = Rise_Control_Type_DISABLE;
//	}
	
}
/**
 * @brief Task计算回调函数
 *
 */
void Class_Chassis::Task_Calculate_PeriodElapsedCallback()
{
	//速度获取及斜波
	Target_Resolution();
    //速度解算
    Inverse_Kinematic();
	
//	Rise_Control();
	//输出限制
	Output();

    //各个电机具体PID
    for (auto& Motor:Motor_Wheel)
    {
        Motor.Task_PID_PeriodElapsedCallback();
    }

//	Motor_Rise.Task_PID_PeriodElapsedCallback();
}

/* ÖÐÑë±ê³ß¸ß¶È±äÁ¿ */
uint16_t y01 = 455;
uint16_t y02 = 420;
uint16_t y03 = 280;
uint16_t y04 = 230;
void Class_Chassis::UI_Task()
{
	Referee.Interaction_Graphic_7.Sender = Referee_Data_Robots_ID_RED_INFANTRY_3;
	Referee.Interaction_Graphic_7.Receiver = Referee_Data_Robots_Client_ID_RED_INFANTRY_3;
	Referee.UI_Draw_Line(&Referee.Interaction_Graphic_7.Graphic_1, (char*)"001", Graphic_Operation_ADD, 0, Graphic_Color_GREEN, 1,  840,   y01,  920,   y01); //µÚÒ»ÐÐ×óºáÏß
	Referee.UI_Draw_Line(&Referee.Interaction_Graphic_7.Graphic_2, (char*)"002", Graphic_Operation_ADD, 0, Graphic_Color_GREEN, 1,  950,   y01,  970,   y01); //µÚÒ»ÐÐÊ®×Öºá
	Referee.UI_Draw_Line(&Referee.Interaction_Graphic_7.Graphic_3, (char*)"003", Graphic_Operation_ADD, 0, Graphic_Color_GREEN, 1, 1000,   y01, 1080,   y01); //µÚÒ»ÐÐÓÒºáÏß
	Referee.UI_Draw_Line(&Referee.Interaction_Graphic_7.Graphic_4, (char*)"004", Graphic_Operation_ADD, 0, Graphic_Color_GREEN, 1,  960,y01-10,  960,y01+10); //µÚÒ»ÐÐÊ®×ÖÊú
	Referee.UI_Draw_Line(&Referee.Interaction_Graphic_7.Graphic_5, (char*)"005", Graphic_Operation_ADD, 0, Graphic_Color_GREEN, 1,  870,   y02,  930,   y02); //µÚ¶þÐÐ×óºáÏß
	Referee.UI_Draw_Line(&Referee.Interaction_Graphic_7.Graphic_6, (char*)"006", Graphic_Operation_ADD, 0, Graphic_Color_GREEN, 5,  959,   y02,  960,   y02); //µÚ¶þÐÐÖÐÐÄµã
	Referee.UI_Draw_Line(&Referee.Interaction_Graphic_7.Graphic_7, (char*)"007", Graphic_Operation_ADD, 0, Graphic_Color_GREEN, 1,  990,   y02, 1050,   y02); //µÚ¶þÐÐÓÒºáÏß
	Referee.Data_Concatenation(Referee_Command_ID_INTERACTION,(uint8_t *)&Referee.Interaction_Graphic_7, sizeof(Referee.Interaction_Graphic_7)-2);
	
	
	
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
