#ifndef OMNI_H
#define OMNI_H

#include "DC_Motor.h"
#include "Dead_Wheel.h"
#include "ROS_mainpp.h"

typedef struct {
	double Vx = 0.0;
	double Vy = 0.0;
	double Omega = 0.0;
} CAR_INFO;

// TODO: Modify for new car - 11
typedef struct {
	double Short = 0.093258; // For Driving Wheel
	double Long = 0.093258;  // For Driving Wheel
	double Sq = 0.093; // For Dead Wheel
} CAR_DIMENSION;

// Omni control
class Omni {
public:
	Omni();

	// Initialize each motor settings.
	void Init();

	// Update each motor's Vnow, Calculate car's Vnow & intergral location with dead wheel encoder.
	void UpdateNowCarInfo_Dead();
	void UpdateCarLocation_Dead();

	// Update each motor's Vnow, Calculate car's Vnow & intergral location with driving wheel encoder.
	void UpdateNowCarInfo_Driving();
	void UpdateCarLocation_Driving();

	double GetMotorVnow(int index);

	// Set GoalCarInfo and Update each motor Vgoal.
	void SetGoalCarInfo(double Vx, double Vy, double Omega);

	// Update each motor's PID value and set PWM.
	void Update_PID();

	// Move the car
	void Move();

	void SetCarRadius(CAR_DIMENSION CarRadius);

	// Debug from Live Expressions
	void SetMotorVgoal();

	// Get car's Vnow & intergral location with dead wheel encoder.
	CAR_INFO GetNowCarInfo_Dead();
	CAR_INFO GetNowCarLocation_Dead();

	// Get car's Vnow & intergral location with driving wheel encoder.
	CAR_INFO GetNowCarInfo_Driving();
	CAR_INFO GetNowCarLocation_Driving();

	// Vgoal
	CAR_INFO GoalCarInfo;

private:

	// Get each motors' V_now
	void UpdateMotorVnow();
	// Get each encoder's V_now
	void UpdateEncoderVnow();

	DC_Motor::Motor motors[4];
	Dead_Wheel::Encoder encoders[4];

	// Vnow & intergral location with dead wheel encoder.
	CAR_INFO NowCarInfo_Dead;
	CAR_INFO NowCarLocation_Dead;

	// Vnow & intergral location with driving wheel encoder.
	CAR_INFO NowCarInfo_Driving;
	CAR_INFO NowCarLocation_Driving;

	// Vgoal
//	CAR_INFO GoalCarInfo;


	// Car Radius
	CAR_DIMENSION CarRadius_;
};

extern CAR_DIMENSION CAR_RADIUS;
extern Omni omni;

#endif /* OMNI_H */
