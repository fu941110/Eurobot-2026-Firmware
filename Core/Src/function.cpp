#include "function.h"

#include "ROS_mainpp.h"
#include "DebugMode.h"

#include "stm32h7xx_hal.h"

#include <algorithm>
#include "Omni.h"

double Vx_goal = 0.0;
double Vy_goal = 0.0;
double Vz_goal = 0.0;
int b = 0;

// ROS spinOnce
//extern TIM_HandleTypeDef htim7;
//
//// Count ROS frequency.
//static int ROS_CAR_FREQUENCY_Driving = 0;
//static int ROS_CAR_FREQUENCY_Dead = 0;
//static int ROS_CAR_FREQUENCY_Stop = 0;
//
//// Publish stopRobot only twice(On/Off)
//static int stop_countdown = 10;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM13) {
		// Update car Vnow & intergral location with driving wheel encoder.
		omni.UpdateNowCarInfo_Driving();
		omni.UpdateCarLocation_Driving();
		b++;
		// Update four wheel's PID value.
		omni.Update_PID();

		// Output GPIO and PWM
		omni.Move();

		// Debug from Live Expressions ( Optional )
		omni.SetMotorVgoal();

		// there is a problem in setting Vgoal
		// hardcode for test
		omni.SetGoalCarInfo(0.02, 0 ,0);
		// omni.SetGoalCarInfo(Vx_goal, Vy_goal, Vz_goal);


//		// ROS pub
//		if (++ROS_CAR_FREQUENCY_Driving >= ROS_CAR_PUB_FREQUENCY) {
//			ROS_CAR_FREQUENCY_Driving = 0;
//			ROS::PubCarVnow_Driving();
//		}
	}
	else
		if(htim->Instance == TIM16) {
		// Update car Vnow & intergral location with dead wheel encoder.
		omni.UpdateNowCarInfo_Dead();
		omni.UpdateCarLocation_Dead();

//		// ROS pub
//		if (++ROS_CAR_FREQUENCY_Dead >= ROS_CAR_PUB_FREQUENCY) {
//			ROS_CAR_FREQUENCY_Dead = 0;
//			ROS::PubCarVnow_Dead();
//		}
	}
//	else if (htim->Instance == TIM7) {
//		ROS::loop();
//		// ROS pub
//		if (++ROS_CAR_FREQUENCY_Stop >= ROS_STOP_PUB_FREQUENCY && stop_countdown) {
//			bool data;
//			if(stop_countdown > 3)	data = true;
//			else	data = false;
//			ROS_CAR_FREQUENCY_Stop = 0;
//			ROS::PubResetNavigation(data);
//			stop_countdown--;
//		}
//	}
}
