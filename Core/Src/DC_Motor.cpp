#include "stm32h7xx_hal.h"
#include "DC_Motor.h"

// Reading encoder's CNT
extern TIM_HandleTypeDef htim13;

// Motor PWM
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim15;

// Encoder
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

int a = 0;

namespace DC_Motor {
double WheelRadius[4] = {0.024, 0.024, 0.024, 0.024};
double ROUND[4] = {2 * WheelRadius[0] * 3.14159, 2 * WheelRadius[1] * 3.14159, 2 * WheelRadius[2] * 3.14159, 2 * WheelRadius[3] * 3.14159};
double CONST_FOR_MOTOR[4] = {ROUND[0] / RES_Ratio_Driving, ROUND[1] / RES_Ratio_Driving, ROUND[2] / RES_Ratio_Driving, ROUND[3] / RES_Ratio_Driving};

void Init() {
	// Init interrupt for reading encoder's CNT
	HAL_TIM_Base_Start_IT(&htim13);

	// Init Encoder
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // Motor[0]
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // Motor[1]
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // Motor[2]
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // Motor[3]

	// Init Motor PWM
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2); // Motor[0]
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1); // Motor[1]
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1); // Motor[2]
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2); // Motor[3]
}

void Motor::Init(short num, TIM_HandleTypeDef *TIM, double P, double I) {
	this->num = num;
	this->TIM = TIM;
	this->P = P;
	this->I = I;
}

// PID controller, the coefficients should be modified.
void Motor::UpdatePI() {
	double p = 0;

	// Record Prev error
	this->error_before = this->error;

	// Count current error
	this->error = this->Vgoal - this->Vnow;

	// Count P
	p = this->P * this->error;

	// Count I
	this->i += this->I * this->error * COUNT_TIME_DRIVING;

	// Limit I in range
	if (this->i > this->I_lim)
		this->i = this->I_lim;
	else if (this->i < 0 - this->I_lim)
		this->i = 0 - this->I_lim;

	// Output = P + I
	this->u = (double) p + this->i;
//	if (num == 0 && isMove == true)
//		this->u = 1.0;
//	else
//		this->u = 0.0;

//	// Filter ( Optional )
//	this->u = this->prev_u * 0.98 + this->u * 0.02;

	// Limit output in rangecontinue_CNT
	if (this->u < -0.6) {
		this->u = -0.6;
	}
	else if (this->u > 0.6) {
		this->u = 0.6;
	}

	// Record Prev output
	this->prev_u = this->u;

	return;
}

void Motor::UpdateVnow() {
	// First, get CNT for this loop
	CNT = __HAL_TIM_GET_COUNTER(TIM);
	a++;
	Record_CNT();
	Vnow = CNT * CONST_FOR_MOTOR[num];  // Unit : m / s

//	if (isMove && num == 0 && DC_index < 2000) {
//		DC_motor_Vnow[DC_index++] = Vnow;
//	}

	// Finally, reset the counter CNT
	__HAL_TIM_SET_COUNTER(TIM, 0);
}

void Motor::SetVgoal(double Vgoal) {
	this->Vgoal = Vgoal;
}

void Motor::Record_CNT() {
	continue_CNT += CNT;
}

double Motor::MoveDis() {
	double dis = continue_CNT * CONST_FOR_MOTOR[num];

	continue_CNT = 0;

	return dis;
}

double Motor::GetVnow() {
	return Vnow;
}

}

